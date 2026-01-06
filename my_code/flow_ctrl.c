#include "flow_ctrl.h"

#include "frn06.h"
#include "pid.h"
#include "tim.h"

/* 入口阶段可选的 boost：用于泵刚启动、流量还没起来时，先短时间给一个较大的占空比。
 * 当前默认关闭（FLOWCTRL_STARTUP_BOOST_MS=0）。
 */
#ifndef FLOWCTRL_STARTUP_BOOST_MS
#define FLOWCTRL_STARTUP_BOOST_MS 0U
#endif

#ifndef FLOWCTRL_STARTUP_BOOST_THRESH_MSLM
#define FLOWCTRL_STARTUP_BOOST_THRESH_MSLM 50
#endif

#ifndef FLOWCTRL_REF_PWM_PERIOD
#define FLOWCTRL_REF_PWM_PERIOD 199U
#endif

#ifndef FLOWCTRL_DEFAULT_KP_REF
#define FLOWCTRL_DEFAULT_KP_REF 120U
#endif

#ifndef FLOWCTRL_DEFAULT_KI_REF
#define FLOWCTRL_DEFAULT_KI_REF 10U
#endif

/* PID->HOLD（保持占空比）的稳定判定：
 * - 连续 N 次误差小于等于 stable_err（mslm）即可进入 HOLD。
 */
#ifndef FLOWCTRL_HOLD_STABLE_N
#define FLOWCTRL_HOLD_STABLE_N 8U
#endif

#ifndef FLOWCTRL_HOLD_STABLE_ERR_MSLM
#define FLOWCTRL_HOLD_STABLE_ERR_MSLM 10L
#endif

#ifndef FLOWCTRL_HOLD_REENTER_ERR_MSLM
#define FLOWCTRL_HOLD_REENTER_ERR_MSLM 35L
#endif

typedef enum
{
  FLOWCTRL_MODE_PID = 0,
  FLOWCTRL_MODE_HOLD = 1
} FlowCtrl_Mode;

static PID_Controller s_pid;
static int32_t s_target_mslm = 1000;
static int32_t s_measured_mslm = 0;
static int32_t s_measured_inst_mslm = 0;
static uint8_t s_measured_filt_inited = 0;
static uint32_t s_pwm_compare = 0;
static uint8_t s_inited = 0;
static uint32_t s_boost_until_tick = 0;

/* 控制模式：
 * - PID：闭环调节
 * - HOLD：锁定某个占空比（s_hold_pwm_compare），直到误差偏离较大再回到 PID
 */
static FlowCtrl_Mode s_mode = FLOWCTRL_MODE_PID;
static uint32_t s_hold_pwm_compare = 0U;
static uint8_t s_stable_cnt = 0U;

static int32_t abs_i32(int32_t x)
{
  return (x >= 0) ? x : -x;
}

void FlowCtrl_Reset(void)
{
  /* 泵开/关或模式切换时调用：清理滤波器、状态机与 PID 内部状态 */
  s_boost_until_tick = 0U;
  s_pwm_compare = 0U;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, s_pwm_compare);
  s_measured_mslm = 0;
  s_measured_inst_mslm = 0;
  s_measured_filt_inited = 0;
  s_mode = FLOWCTRL_MODE_PID;
  s_hold_pwm_compare = 0U;
  s_stable_cnt = 0U;
  PID_Reset(&s_pid);
}

void FlowCtrl_Sample(void)
{
  int32_t flow;

  if (!s_inited)
  {
    (void)FlowCtrl_Init();
  }

  if (FRN06_ReadFlow_mslm(&flow) == HAL_OK)
  {
    s_measured_inst_mslm = flow;
    if (s_measured_filt_inited == 0U)
    {
      s_measured_mslm = flow;
      s_measured_filt_inited = 1U;
    }
    else
    {
      /* 简单一阶 IIR：抑制传感器抖动，避免 PID 输出抖动 */
      s_measured_mslm = s_measured_mslm + ((flow - s_measured_mslm) / 5);
    }
  }
}

HAL_StatusTypeDef FlowCtrl_Init(void)
{
  uint32_t period;
  uint32_t scale_num;
  uint32_t kp;
  uint32_t ki;

  s_pwm_compare = 0U;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, s_pwm_compare);

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  period = (uint32_t)htim4.Init.Period;

  scale_num = (period + 1U);
  if (scale_num < 1U)
  {
    scale_num = 1U;
  }

  kp = (uint32_t)((FLOWCTRL_DEFAULT_KP_REF * scale_num + (FLOWCTRL_REF_PWM_PERIOD + 1U) / 2U) / (FLOWCTRL_REF_PWM_PERIOD + 1U));
  if (kp < 1U)
  {
    kp = 1U;
  }

  ki = (uint32_t)((FLOWCTRL_DEFAULT_KI_REF * scale_num + (FLOWCTRL_REF_PWM_PERIOD + 1U) / 2U) / (FLOWCTRL_REF_PWM_PERIOD + 1U));
  if (ki < 1U)
  {
    ki = 1U;
  }

  PID_Init(&s_pid, (int32_t)kp, (int32_t)ki, 0, 0, (int32_t)period);
  PID_SetIntegralLimit(&s_pid, 0, (int32_t)period);

  FlowCtrl_Reset();

  s_inited = 1;
  return HAL_OK;
}

void FlowCtrl_SetTarget_mslm(int32_t target_mslm)
{
  s_target_mslm = target_mslm;
}

void FlowCtrl_SetTunings(int32_t kp, int32_t ki, int32_t kd)
{
  uint32_t period = (uint32_t)htim4.Init.Period;
  s_pid.kp = kp;
  s_pid.ki = ki;
  s_pid.kd = kd;
  s_pid.out_min = 0;
  s_pid.out_max = (int32_t)period;
  s_pid.i_min = 0;
  s_pid.i_max = (int32_t)period;
}

void FlowCtrl_Update(void)
{
  uint32_t now;

  if (!s_inited)
  {
    (void)FlowCtrl_Init();
  }

  FlowCtrl_Sample();

  now = HAL_GetTick();

  if (s_mode == FLOWCTRL_MODE_HOLD)
  {
    int32_t err = s_target_mslm - s_measured_mslm;
    if (abs_i32(err) >= (int32_t)FLOWCTRL_HOLD_REENTER_ERR_MSLM)
    {
      /* HOLD->PID：误差偏离过大，回到 PID 调节。
       * 关键点：为了避免“回到 PID 的第一下输出掉到 0”（看起来像泵瞬间关了一下），
       * 这里不再 PID_Reset() 清零，而是用当前 HOLD 的占空比作为 PID 初始输出基准。
       */
      s_mode = FLOWCTRL_MODE_PID;
      s_stable_cnt = 0U;

      {
        /* 让下一次 PID_Update 的输出尽量等于当前占空比：out = P + I + D。
         * - 先令 prev_err=err，使 D≈0
         * - 再设置 I = desired_out - P，并限制在积分限幅内
         */
        int32_t desired_out = (int32_t)s_hold_pwm_compare;
        int32_t p = (int32_t)((int64_t)s_pid.kp * (int64_t)err / 1000LL);
        int32_t i = desired_out - p;

        if (desired_out < s_pid.out_min)
        {
          desired_out = s_pid.out_min;
        }
        if (desired_out > s_pid.out_max)
        {
          desired_out = s_pid.out_max;
        }

        if (i < s_pid.i_min)
        {
          i = s_pid.i_min;
        }
        if (i > s_pid.i_max)
        {
          i = s_pid.i_max;
        }

        s_pid.i_state = i;
        s_pid.prev_err = err;
      }
    }
    else
    {
      /* HOLD：保持占空比不变 */
      s_pwm_compare = s_hold_pwm_compare;
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, s_pwm_compare);
      return;
    }
  }

  if (s_target_mslm > 0 && s_measured_mslm < (int32_t)FLOWCTRL_STARTUP_BOOST_THRESH_MSLM)
  {
    /* 可选的启动 boost：目标>0 且流量还很低时，开始计时 boost */
    if (s_boost_until_tick == 0U)
    {
      s_boost_until_tick = now + (uint32_t)FLOWCTRL_STARTUP_BOOST_MS;
    }
  }
  else
  {
    s_boost_until_tick = 0U;
  }

  if (s_boost_until_tick != 0U && (int32_t)(now - s_boost_until_tick) < 0)
  {
    s_pwm_compare = (uint32_t)htim4.Init.Period;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, s_pwm_compare);
    return;
  }

  {
    /* PID 输出到 PWM：PID 输出范围已经配置为 [0, period] */
    int32_t out = PID_Update(&s_pid, s_target_mslm, s_measured_mslm);
    if (out < 0)
    {
      out = 0;
    }
    s_pwm_compare = (uint32_t)out;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, s_pwm_compare);
  }

  {
    /* 稳定判定：误差连续满足阈值才进入 HOLD */
    int32_t err = s_target_mslm - s_measured_mslm;
    if (abs_i32(err) <= (int32_t)FLOWCTRL_HOLD_STABLE_ERR_MSLM)
    {
      if (s_stable_cnt < 255U)
      {
        s_stable_cnt++;
      }
    }
    else
    {
      s_stable_cnt = 0U;
    }

    if (s_stable_cnt >= (uint8_t)FLOWCTRL_HOLD_STABLE_N)
    {
      /* PID->HOLD：锁定当前占空比 */
      s_mode = FLOWCTRL_MODE_HOLD;
      s_hold_pwm_compare = s_pwm_compare;
      s_stable_cnt = 0U;
    }
  }
}

int32_t FlowCtrl_GetTarget_mslm(void)
{
  return s_target_mslm;
}

int32_t FlowCtrl_GetMeasured_mslm(void)
{
  return s_measured_mslm;
}

uint32_t FlowCtrl_GetPwmCompare(void)
{
  return s_pwm_compare;
}
