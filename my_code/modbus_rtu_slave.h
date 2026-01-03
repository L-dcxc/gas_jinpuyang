#ifndef MODBUS_RTU_SLAVE_H
#define MODBUS_RTU_SLAVE_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void ModbusRTUSlave_Init(UART_HandleTypeDef *huart, uint8_t slave_id);

void ModbusRTUSlave_OnByte(uint8_t b);
void ModbusRTUSlave_OnIdle(void);
void ModbusRTUSlave_Poll(void);

void ModbusRTUSlave_SetGasType(uint16_t gas_type);
void ModbusRTUSlave_SetConcentrationFloat(float conc);

uint16_t ModbusRTUSlave_GetGasType(void);
float ModbusRTUSlave_GetConcentrationFloat(void);

#ifdef __cplusplus
}
#endif

#endif
