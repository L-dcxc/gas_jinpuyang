#ifndef MODBUS_RTU_SLAVE_H
#define MODBUS_RTU_SLAVE_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void ModbusRTUSlave_Init(UART_HandleTypeDef *huart, uint8_t slave_id);

void ModbusRTUSlave_OnByteFromUart(UART_HandleTypeDef *huart, uint8_t b);
void ModbusRTUSlave_OnIdleFromUart(UART_HandleTypeDef *huart);
void ModbusRTUSlave_PollAll(void);

void ModbusRTUSlave_SetConcentrationU16(uint8_t ch, uint16_t conc);
uint16_t ModbusRTUSlave_GetConcentrationU16(uint8_t ch);

void ModbusRTUSlave_SetPumpEnable(uint16_t en);
uint16_t ModbusRTUSlave_GetPumpEnable(void);

#ifdef __cplusplus
}
#endif

#endif
