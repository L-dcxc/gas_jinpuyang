/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "ads1256.h"
#include "frn06.h"
#include "flow_ctrl.h"
#include "modbus_rtu_slave.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USART1 收到任何字节后回 "ok\r\n"（仅用于最初串口连通性测试，默认关闭） */
#define UART1_RX_REPLY_OK 0

/* printf(USART3) 的单次发送超时，避免调试打印长时间阻塞主循环 */
#define DEBUG_UART_TX_TIMEOUT_MS 5U

/* 仅做 Modbus 联调时可置 1：关闭 ADS/FRN06/FlowCtrl 等，避免外设影响 */
#define MODBUS_ONLY_TEST 0

/* 泄漏传感器（PA0/ADC1_IN0）原始 ADC -> 电压（mV）换算用参数 */
#define LEAK_ADC_VREF_MV 3300U
#define LEAK_ADC_FULL_SCALE 4095U

/* 泄漏传感器浓度标定（按 ADC 原始值线性映射）
 * - ADC=2000 -> 0%LEL
 * - ADC=2100 -> 100%LEL
 * 输出单位：%LEL x100（0~10000）
 */
#define LEAK_ADC_ZERO 2000U
#define LEAK_ADC_FULL 2100U

/* 浓度单位：%LEL x100（100.00%LEL -> 10000） */
#define LEAK_CONC_MAX_U16 10000U

/* 三态报警阈值（同样是 %LEL x100）
 * - >25% 低报（1）
 * - >50% 高报（2）
 */
#define LEAK_ALARM_LOW_TH 2500U
#define LEAK_ALARM_HIGH_TH 5000U  //高低报阈值

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Modbus 接收中断：每次只收 1 字节（UART IRQ 中不断续接收） */
uint8_t uart1_rx_byte;
uint8_t rs485_rx_byte;
uint8_t uart1_rx_seen;

/* 通信统计（用于调试串口/485 是否有数据进来） */
uint32_t uart1_rx_cnt;
uint32_t uart2_rx_cnt;

/* 轮询定时：FRN06 打印 / FlowCtrl 更新 / ADS 打印等 */
uint32_t frn06_test_last_tick = 0;
uint32_t flow_ctrl_last_tick = 0;
uint32_t ads_print_last_tick = 0;

/* 泵使能的上一次状态：用于检测 0->1 或 1->0 变化，触发 FlowCtrl_Reset() */
uint16_t pump_enable_last = 0;

/* MODBUS_ONLY_TEST 模式下的调试打印节拍 */
uint32_t modbus_dbg_last_tick = 0;

/* PA0 泄漏传感器采样/打印节拍与结果缓存 */
uint32_t leak_adc_last_tick = 0;
uint32_t leak_print_last_tick = 0;
uint16_t leak_adc_raw = 0;
uint16_t leak_mv = 0;

/* leak_conc_u16：%LEL x100（0~10000）；leak_state_u16：0正常/1低报/2高报 */
uint16_t leak_conc_u16 = 0;
uint16_t leak_state_u16 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ADS1256 电压(uV) -> 浓度(%LEL x100) 的线性换算：0.4V=0，2.0V=10000 */
static uint16_t ADS_uV_To_LelX100(int32_t uv)
{
  const int32_t zero_uv = 400000;
  const int32_t full_uv = 2000000;
  const int32_t span_uv = (full_uv - zero_uv);

  if (uv <= zero_uv)
  {
    return 0U;
  }
  if (uv >= full_uv)
  {
    return 10000U;
  }
  if (span_uv <= 0)
  {
    return 0U;
  }

  {
    int64_t num = (int64_t)(uv - zero_uv) * 10000LL;
    int64_t den = (int64_t)span_uv;
    num += den / 2;
    {
      int64_t out = num / den;
      if (out < 0)
      {
        out = 0;
      }
      if (out > 10000)
      {
        out = 10000;
      }
      return (uint16_t)out;
    }
  }
}
/**
  * @brief 串口接收完成中断回调函数
  * @note  当前仅对 USART1 做回显测试：收到 1 字节就立即回发。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        uart1_rx_seen = 1U;
        uart1_rx_cnt++;
        ModbusRTUSlave_OnByteFromUart(&huart1, uart1_rx_byte);
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
    else if (huart->Instance == USART2)
    {
        uart2_rx_cnt++;
        ModbusRTUSlave_OnByteFromUart(&huart2, rs485_rx_byte);
        HAL_UART_Receive_IT(&huart2, &rs485_rx_byte, 1);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return;
  }

  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    (void)HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  }
  else if (huart->Instance == USART2)
  {
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    __HAL_UART_CLEAR_FEFLAG(&huart2);
    __HAL_UART_CLEAR_NEFLAG(&huart2);
    __HAL_UART_CLEAR_PEFLAG(&huart2);
    (void)HAL_UART_Receive_IT(&huart2, &rs485_rx_byte, 1);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ADS_DRDY_Pin)
  {
    /* ADS1256 DRDY 下降沿：提示有新数据可读（由 ADS1256_Update() 消费此标志） */
    g_ads1256_drdy_flag = 1U;
  }
}

/* PA0 原始 ADC -> mV：仅用于打印观察输入电压，控制逻辑不依赖该值 */
static uint16_t LeakAdc_To_mV(uint16_t adc)
{
  uint32_t mv = ((uint32_t)adc * (uint32_t)LEAK_ADC_VREF_MV + (LEAK_ADC_FULL_SCALE / 2U)) / (uint32_t)LEAK_ADC_FULL_SCALE;
  if (mv > 65535U)
  {
    mv = 65535U;
  }
  return (uint16_t)mv;
}

static uint16_t LeakAdc_To_ConcU16(uint16_t adc)
{
  /* 基于标定点的线性插值：2000->0%LEL，2100->100%LEL（输出%LEL x100） */
  if (adc <= (uint16_t)LEAK_ADC_ZERO)
  {
    return 0U;
  }
  if (adc >= (uint16_t)LEAK_ADC_FULL)
  {
    return (uint16_t)LEAK_CONC_MAX_U16;
  }
  {
    uint32_t span = (uint32_t)((uint16_t)LEAK_ADC_FULL - (uint16_t)LEAK_ADC_ZERO);
    uint32_t num = (uint32_t)(adc - (uint16_t)LEAK_ADC_ZERO) * (uint32_t)LEAK_CONC_MAX_U16;
    uint32_t out = (span == 0U) ? 0U : ((num + span / 2U) / span);
    if (out > (uint32_t)LEAK_CONC_MAX_U16)
    {
      out = (uint32_t)LEAK_CONC_MAX_U16;
    }
    return (uint16_t)out;
  }
}

static uint16_t LeakConc_To_StateU16(uint16_t conc_u16)
{
  /* 三态报警：0正常/1低报/2高报；同时点亮/熄灭板载 LED(示意) */
  if (conc_u16 > (uint16_t)LEAK_ALARM_HIGH_TH)
  {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);//开灯
    return 2U;
  }
  if (conc_u16 > (uint16_t)LEAK_ALARM_LOW_TH)
  {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    return 1U;
  }
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);//关灯
  return 0U;
}

/**
  * @brief printf 重定向到调试串口 USART3
  * @note  适用于 Keil/ARMCC 常见 printf 实现。
  */
int fputc(int ch, FILE *f)
{
  (void)f;
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, DEBUG_UART_TX_TIMEOUT_MS);
  return ch;
}

/**
  * @brief 兼容 GCC/newlib 的 printf 重定向（部分工程会走 _write）
  */
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
  (void)file;
  (void)HAL_UART_Transmit(&huart3, (uint8_t *)ptr, (uint16_t)len, DEBUG_UART_TX_TIMEOUT_MS);
  return len;
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* Modbus 从机：同时挂载到 USART1/USART2，两路共用同一份保持寄存器数据
   * - USART1：通常接串口屏/上位机
   * - USART2：通常接 RS485
   */
  ModbusRTUSlave_Init(&huart1, 1);
  ModbusRTUSlave_Init(&huart2, 1);

  /* 采用中断方式逐字节接收，接收回调里喂给 Modbus 协议栈 */
  HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &rs485_rx_byte, 1);

  /* NVIC 优先级：让 Modbus 接收优先级高于 ADS 打印，避免丢帧 */
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_EnableIRQ(USART3_IRQn);

#if MODBUS_ONLY_TEST
  HAL_NVIC_DisableIRQ(EXTI0_IRQn);

  ModbusRTUSlave_SetPumpEnable(0U);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  (void)HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
  }
#endif

  HAL_SPI_DeInit(&hspi1);
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  (void)HAL_SPI_Init(&hspi1);

  /* ADS/RS485 相关 GPIO 上电默认电平：
   * - CS 空闲建议拉高（不选中）
   * - RESET 释放建议拉高（退出复位）
   * - RS485_DE 保持低（接收模式），当前未接485也不影响
   */
  HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
  printf("open is OK!!!\r\n");
  /* 为了避免线路未接/接触不良导致“浮空随机值”，这里在运行时做一次 GPIO 重新配置：
   * - DRDY：输入上拉（未接时稳定为 1；接上后由 ADS1256 驱动）
   * - SPI1 MISO(PA6)：AF 无上下拉（由 ADS1256 DOUT 推挽驱动）
   */
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

#if !MODBUS_ONLY_TEST
  FRN06_Init();
  {
    /* FRN06 传感器参数（offset/scale）上电读一次，后续用来做流量单位换算 */
    int32_t off = 0;
    int32_t scale = 0;
    if (FRN06_ReadParams(&off, &scale) == HAL_OK)
    {
      printf("FRN06 offset=%ld, scale=%ld\r\n", (long)off, (long)scale);
    }
    else
    {
      printf("FRN06 read params failed\r\n");
    }
  }

  (void)FlowCtrl_Init();
  /* 默认目标流量：500 mslm（=0.500 slm），可后续改为从 Modbus/界面下发 */
  FlowCtrl_SetTarget_mslm(500);

  /* 板载指示灯：PD12 作为电源/运行灯 */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);//电源灯
  //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#if !MODBUS_ONLY_TEST
    /* PA0 泄漏传感器采样：100ms 采一次（ADC 单次转换方式）
     * - leak_conc_u16：浓度（%LEL x100）
     * - leak_state_u16：三态报警（0/1/2）并发布到 Modbus 寄存器
     */
    if ((HAL_GetTick() - leak_adc_last_tick) >= 100U)
    {
      leak_adc_last_tick = HAL_GetTick();
      if (HAL_ADC_Start(&hadc1) == HAL_OK)
      {
        if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK)
        {
          uint32_t v = HAL_ADC_GetValue(&hadc1);
          if (v > 0xFFFFU)
          {
            v = 0xFFFFU;
          }
          leak_adc_raw = (uint16_t)v;
          leak_mv = LeakAdc_To_mV(leak_adc_raw);
          leak_conc_u16 = LeakAdc_To_ConcU16(leak_adc_raw);
          leak_state_u16 = LeakConc_To_StateU16(leak_conc_u16);
          ModbusRTUSlave_SetLeakState(leak_state_u16);
        }
        (void)HAL_ADC_Stop(&hadc1);
      }
    }

    /* 低频打印，避免太多 printf 影响实时控制 */
    if ((HAL_GetTick() - leak_print_last_tick) >= 500U)
    {
      leak_print_last_tick = HAL_GetTick();
      printf("LEAK adc=%u mv=%u conc=%u state=%u\r\n", leak_adc_raw, leak_mv, leak_conc_u16, leak_state_u16);
    }
#endif

#if !MODBUS_ONLY_TEST
    /* ADS1256 采样更新：驱动内部使用 DRDY 标志做非阻塞采样
     * 这里每次循环都调用 Update，然后将四路浓度写入 Modbus 保持寄存器 0~3。
     */
    ADS1256_Update();

    {
      uint16_t conc_u16[4];
      for (uint8_t ch = 0; ch < 4U; ch++)
      {
        int32_t uv = ADS1256_GetLatestUv(ch);
        if (uv == (int32_t)0x80000000 || uv == (int32_t)0x80000001)
        {
          conc_u16[ch] = 0U;
        }
        else
        {
          conc_u16[ch] = ADS_uV_To_LelX100(uv);
        }
        ModbusRTUSlave_SetConcentrationU16(ch, conc_u16[ch]);
      }
    }
#endif

    /* Modbus 协议栈轮询：解析接收帧并发送响应（两路串口都在这里处理） */
    ModbusRTUSlave_PollAll();

#if MODBUS_ONLY_TEST
    if ((HAL_GetTick() - modbus_dbg_last_tick) >= 1000U)
    {
      modbus_dbg_last_tick = HAL_GetTick();
      printf("MB rx1=%lu rx2=%lu ok=%lu bad=%lu\r\n",
             (unsigned long)uart1_rx_cnt,
             (unsigned long)uart2_rx_cnt,
             (unsigned long)g_modbus_rx_ok_frames,
             (unsigned long)g_modbus_rx_bad_crc_frames);
    }
#endif

#if UART1_RX_REPLY_OK
    if (uart1_rx_seen != 0U)
    {
      static const uint8_t ok_msg[] = "ok\r\n";
      uart1_rx_seen = 0U;
      (void)HAL_UART_Transmit(&huart1, (uint8_t *)ok_msg, (uint16_t)(sizeof(ok_msg) - 1U), 10);
    }
#endif

#if !MODBUS_ONLY_TEST
    /* 流量闭环控制：50ms 更新一次
     * - pump_en 来自 Modbus 寄存器 4（可写），用于开/关泵
     * - 使能变化时 reset，避免积分项/状态残留
     */
    if ((HAL_GetTick() - flow_ctrl_last_tick) >= 50U)
    {
      flow_ctrl_last_tick = HAL_GetTick();
      {
        uint16_t pump_en = ModbusRTUSlave_GetPumpEnable();

        if (pump_en != pump_enable_last)
        {
          pump_enable_last = pump_en;
          FlowCtrl_Reset();
        }

        if (pump_en != 0U)
        {
          FlowCtrl_Update();
        }
        else
        {
          FlowCtrl_Sample();
          __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        }
      }
    }
#endif

#if !MODBUS_ONLY_TEST
    /* 每 1 秒打印一次流量传感器读数与 PWM 输出，用于现场观察稳定性 */
    if ((HAL_GetTick() - frn06_test_last_tick) >= 1000U)
    {
      frn06_test_last_tick = HAL_GetTick();
      {
        int32_t meas = 0;
        int32_t filt = FlowCtrl_GetMeasured_mslm();
        uint32_t pwm = FlowCtrl_GetPwmCompare();
        int32_t raw = 0;
        (void)FRN06_ReadFlow_mslm(&meas);
        (void)FRN06_ReadFlowRaw(&raw);

        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim4);
        uint32_t period = arr + 1U;
        uint32_t duty_x1000 = 0;
        if (period > 0U)
        {
          duty_x1000 = (uint32_t)((pwm * 1000U) / period);
        }

        int32_t abs_meas = (meas >= 0) ? meas : -meas;

        int32_t abs_filt = (filt >= 0) ? filt : -filt;

        printf("FLOW raw=%ld inst=%c%ld.%03ld filt=%c%ld.%03ld pwm=%lu/%lu (duty=%lu.%01lu%%)\r\n",
               (long)raw,
               (meas < 0) ? '-' : '+',
               (long)(abs_meas / 1000L),
               (long)(abs_meas % 1000L),
               (filt < 0) ? '-' : '+',
               (long)(abs_filt / 1000L),
               (long)(abs_filt % 1000L),
               (unsigned long)pwm,
               (unsigned long)arr,
               (unsigned long)(duty_x1000 / 10U),
               (unsigned long)(duty_x1000 % 10U));
      }
    }
#endif

#if !MODBUS_ONLY_TEST
    /* ADS 调试打印：每 200ms 输出一次（raw/uV/conc），便于校准与排错 */
    if ((HAL_GetTick() - ads_print_last_tick) >= 200U)
    {
      ads_print_last_tick = HAL_GetTick();
      {
        int32_t raw[4];
        int32_t uv[4];
        uint16_t conc_u16[4];
        GPIO_PinState drdy = HAL_GPIO_ReadPin(ADS_DRDY_GPIO_Port, ADS_DRDY_Pin);
        uint8_t status = ADS1256_ReadStatus();

        for (uint8_t ch = 0; ch < 4U; ch++)
        {
          raw[ch] = ADS1256_GetLatestRaw(ch);
          uv[ch] = ADS1256_GetLatestUv(ch);
          if (uv[ch] == (int32_t)0x80000000 || uv[ch] == (int32_t)0x80000001)
          {
            conc_u16[ch] = 0U;
          }
          else
          {
            conc_u16[ch] = ADS_uV_To_LelX100(uv[ch]);
          }
        }

        printf("ADS drdy=%d status=0x%02X\r\n", (int)drdy, status);
        printf("ADS raw : ch0=%ld ch1=%ld ch2=%ld ch3=%ld\r\n",
               (long)raw[0], (long)raw[1], (long)raw[2], (long)raw[3]);
        printf("ADS conc: ch0=%u ch1=%u ch2=%u ch3=%u\r\n",
               (unsigned int)conc_u16[0], (unsigned int)conc_u16[1], (unsigned int)conc_u16[2], (unsigned int)conc_u16[3]);
        printf("ADS volt(uV): ch0=%ld ch1=%ld ch2=%ld ch3=%ld\r\n",
               (long)uv[0], (long)uv[1], (long)uv[2], (long)uv[3]);
      }
    }
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
