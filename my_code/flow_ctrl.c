#include "flow_ctrl.h"

#include "frn06.h"
#include "pid.h"
#include "tim.h"

#ifndef FLOWCTRL_STARTUP_BOOST_MS
#define FLOWCTRL_STARTUP_BOOST_MS 500U
#endif

#ifndef FLOWCTRL_STARTUP_BOOST_THRESH_MSLM
#define FLOWCTRL_STARTUP_BOOST_THRESH_MSLM 50
#endif

static PID_Controller s_pid;
static int32_t s_target_mslm = 1000;
static int32_t s_measured_mslm = 0;
static uint32_t s_pwm_compare = 0;
static uint8_t s_inited = 0;
static uint32_t s_boost_until_tick = 0;

HAL_StatusTypeDef FlowCtrl_Init(void)
{
  uint32_t period;

  if (HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }

  period = (uint32_t)htim4.Init.Period;
  PID_Init(&s_pid, 50, 1, 0, 0, (int32_t)period);
  PID_SetIntegralLimit(&s_pid, 0, (int32_t)period);

  s_pwm_compare = 0;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, s_pwm_compare);

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
  int32_t flow;
  uint32_t now;

  if (!s_inited)
  {
    (void)FlowCtrl_Init();
  }

  if (FRN06_ReadFlow_mslm(&flow) == HAL_OK)
  {
    s_measured_mslm = flow;
  }

  now = HAL_GetTick();
  if (s_target_mslm > 0 && s_measured_mslm < (int32_t)FLOWCTRL_STARTUP_BOOST_THRESH_MSLM)
  {
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
    int32_t out = PID_Update(&s_pid, s_target_mslm, s_measured_mslm);
    if (out < 0)
    {
      out = 0;
    }
    s_pwm_compare = (uint32_t)out;
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, s_pwm_compare);
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
