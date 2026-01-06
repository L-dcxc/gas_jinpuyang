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

void ModbusRTUSlave_SetLeakState(uint16_t state);
uint16_t ModbusRTUSlave_GetLeakState(void);

uint16_t ModbusRTUSlave_GetZeroCalibReq(void);
void ModbusRTUSlave_ClearZeroCalibReq(void);
void ModbusRTUSlave_SetZeroCalibResult(uint16_t result);
uint16_t ModbusRTUSlave_GetZeroCalibResult(void);

extern volatile uint32_t g_modbus_rx_ok_frames;
extern volatile uint32_t g_modbus_rx_bad_crc_frames;

#ifdef __cplusplus
}
#endif

#endif
