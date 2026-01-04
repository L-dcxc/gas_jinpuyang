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

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t uart1_rx_byte;
uint8_t rs485_rx_byte;
uint32_t frn06_test_last_tick = 0;
uint32_t flow_ctrl_last_tick = 0;
uint32_t ads_print_last_tick = 0;
uint16_t pump_enable_last = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief 串口接收完成中断回调函数
  * @note  当前仅对 USART1 做回显测试：收到 1 字节就立即回发。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        ModbusRTUSlave_OnByteFromUart(&huart1, uart1_rx_byte);
        HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
    }
    else if (huart->Instance == USART2)
    {
        ModbusRTUSlave_OnByteFromUart(&huart2, rs485_rx_byte);
        HAL_UART_Receive_IT(&huart2, &rs485_rx_byte, 1);
    }
}

/**
  * @brief printf 重定向到调试串口 USART3
  * @note  适用于 Keil/ARMCC 常见 printf 实现。
  */
int fputc(int ch, FILE *f)
{
  (void)f;
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/**
  * @brief 兼容 GCC/newlib 的 printf 重定向（部分工程会走 _write）
  */
#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart3, (uint8_t *)ptr, (uint16_t)len, 0xFFFF);
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
  ModbusRTUSlave_Init(&huart1, 1);
  ModbusRTUSlave_Init(&huart2, 1);

  HAL_UART_Receive_IT(&huart1, &uart1_rx_byte, 1);
  HAL_UART_Receive_IT(&huart2, &rs485_rx_byte, 1);
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

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

    GPIO_InitStruct.Pin = ADS_DRDY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(ADS_DRDY_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }

  FRN06_Init();
  (void)FlowCtrl_Init();
  FlowCtrl_SetTarget_mslm(1000);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ADS1256_Update();

    {
      float conc_f[4];
      uint16_t conc_u16[4];
      for (uint8_t ch = 0; ch < 4U; ch++)
      {
        int32_t uv = ADS1256_GetLatestUv(ch);
        float v = (float)uv / 1000000.0f;
        float conc;
        if (v <= 0.4f)
        {
          conc = 0.0f;
        }
        else if (v >= 2.0f)
        {
          conc = 10000.0f;
        }
        else
        {
          conc = (v - 0.4f) * (10000.0f / 1.6f);
        }
        conc_f[ch] = conc;

        if (conc <= 0.0f)
        {
          conc_u16[ch] = 0U;
        }
        else if (conc >= 65535.0f)
        {
          conc_u16[ch] = 65535U;
        }
        else
        {
          conc_u16[ch] = (uint16_t)(conc + 0.5f);
        }
        ModbusRTUSlave_SetConcentrationU16(ch, conc_u16[ch]);
      }
    }

    ModbusRTUSlave_PollAll();

    if ((HAL_GetTick() - flow_ctrl_last_tick) >= 100U)
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
          __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
        }
      }
    }

    if ((HAL_GetTick() - frn06_test_last_tick) >= 1000U)
    {
      frn06_test_last_tick = HAL_GetTick();
      {
        int32_t target = FlowCtrl_GetTarget_mslm();
        int32_t meas = FlowCtrl_GetMeasured_mslm();
        uint32_t pwm = FlowCtrl_GetPwmCompare();
        int32_t raw = 0;
        (void)FRN06_ReadFlowRaw(&raw);
        uint32_t arr = (uint32_t)htim4.Init.Period;
        uint32_t duty_x1000 = 0;
        if ((arr + 1U) != 0U)
        {
          duty_x1000 = (uint32_t)(((uint64_t)pwm * 1000ULL + (uint64_t)(arr + 1U) / 2ULL) / (uint64_t)(arr + 1U));
        }
        int32_t abs_target = (target >= 0) ? target : -target;
        int32_t abs_meas = (meas >= 0) ? meas : -meas;
        printf("FLOW raw=%ld, target=%c%ld.%03ld SLM, meas=%c%ld.%03ld SLM, pwm=%lu (ARR=%lu, duty=%lu.%01lu%%)\r\n",
               (long)raw,
               (target < 0) ? '-' : '+',
               (long)(abs_target / 1000L),
               (long)(abs_target % 1000L),
               (meas < 0) ? '-' : '+',
               (long)(abs_meas / 1000L),
               (long)(abs_meas % 1000L),
               (unsigned long)pwm,
               (unsigned long)arr,
               (unsigned long)(duty_x1000 / 10U),
               (unsigned long)(duty_x1000 % 10U));
      }
    }

    if ((HAL_GetTick() - ads_print_last_tick) >= 200U)
    {
      ads_print_last_tick = HAL_GetTick();
      {
        int32_t raw[4];
        int32_t uv[4];
        float v[4];
        float conc[4];
        GPIO_PinState drdy = HAL_GPIO_ReadPin(ADS_DRDY_GPIO_Port, ADS_DRDY_Pin);
        uint8_t status = ADS1256_ReadStatus();

        for (uint8_t ch = 0; ch < 4U; ch++)
        {
          raw[ch] = ADS1256_GetLatestRaw(ch);
          uv[ch] = ADS1256_GetLatestUv(ch);
          v[ch] = (float)uv[ch] / 1000000.0f;
          if (v[ch] <= 0.4f)
          {
            conc[ch] = 0.0f;
          }
          else if (v[ch] >= 2.0f)
          {
            conc[ch] = 10000.0f;
          }
          else
          {
            conc[ch] = (v[ch] - 0.4f) * (10000.0f / 1.6f);
          }
        }

        printf("ADS drdy=%d status=0x%02X\r\n", (int)drdy, status);
        printf("ADS raw : ch0=%ld ch1=%ld ch2=%ld ch3=%ld\r\n",
               (long)raw[0], (long)raw[1], (long)raw[2], (long)raw[3]);
        printf("ADS conc: ch0=%0.2f ch1=%0.2f ch2=%0.2f ch3=%0.2f\r\n",
               conc[0], conc[1], conc[2], conc[3]);
        printf("ADS volt: ch0=%0.6f ch1=%0.6f ch2=%0.6f ch3=%0.6f\r\n",
               v[0], v[1], v[2], v[3]);
      }
    }
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
