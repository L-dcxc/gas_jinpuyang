#include "modbus_rtu_slave.h"

#ifndef MODBUS_RTU_MAX_FRAME
#define MODBUS_RTU_MAX_FRAME 256
#endif

#ifndef MODBUS_RTU_RX_SILENT_TIMEOUT_MS
#define MODBUS_RTU_RX_SILENT_TIMEOUT_MS 10U
#endif

#define MODBUS_FUNC_READ_HOLDING_REGS 0x03
#define MODBUS_FUNC_WRITE_SINGLE_REG  0x06

#define MODBUS_EX_ILLEGAL_FUNCTION    0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDR   0x02
#define MODBUS_EX_ILLEGAL_DATA_VALUE  0x03

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_slave_id = 1;

static volatile uint8_t s_rx_buf[MODBUS_RTU_MAX_FRAME];
static volatile uint16_t s_rx_len = 0;
static volatile uint32_t s_last_rx_tick = 0;
static volatile uint8_t s_frame_ready = 0;

static uint16_t s_gas_type = 0;
static float s_conc = 0.0f;

static uint16_t crc16_modbus(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x0001)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  return crc;
}

static void u16_be_put(uint8_t *p, uint16_t v)
{
  p[0] = (uint8_t)((v >> 8) & 0xFF);
  p[1] = (uint8_t)(v & 0xFF);
}

static uint16_t u16_be_get(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static void send_exception(uint8_t func, uint8_t ex)
{
  uint8_t out[5];
  out[0] = s_slave_id;
  out[1] = (uint8_t)(func | 0x80);
  out[2] = ex;
  uint16_t crc = crc16_modbus(out, 3);
  out[3] = (uint8_t)(crc & 0xFF);
  out[4] = (uint8_t)((crc >> 8) & 0xFF);
  (void)HAL_UART_Transmit(s_huart, out, sizeof(out), 100);
}

static int read_holding_reg(uint16_t addr_0based, uint16_t *out)
{
  if (addr_0based == 0)
  {
    *out = s_gas_type;
    return 1;
  }
  else if (addr_0based == 1 || addr_0based == 2)
  {
    union {
      float f;
      uint32_t u32;
    } u;
    u.f = s_conc;

    uint16_t hi = (uint16_t)((u.u32 >> 16) & 0xFFFF);
    uint16_t lo = (uint16_t)(u.u32 & 0xFFFF);

    *out = (addr_0based == 1) ? hi : lo;
    return 1;
  }
  return 0;
}

static int write_holding_reg(uint16_t addr_0based, uint16_t val)
{
  if (addr_0based == 0)
  {
    s_gas_type = val;
    return 1;
  }
  return 0;
}

static void handle_read_holding_regs(const uint8_t *frame, uint16_t len)
{
  (void)len;
  uint16_t start = u16_be_get(&frame[2]);
  uint16_t qty = u16_be_get(&frame[4]);

  if (qty == 0 || qty > 125)
  {
    send_exception(frame[1], MODBUS_EX_ILLEGAL_DATA_VALUE);
    return;
  }

  uint8_t out[3 + 2 * 125 + 2];
  out[0] = s_slave_id;
  out[1] = MODBUS_FUNC_READ_HOLDING_REGS;
  out[2] = (uint8_t)(qty * 2);

  for (uint16_t i = 0; i < qty; i++)
  {
    uint16_t reg;
    if (!read_holding_reg((uint16_t)(start + i), &reg))
    {
      send_exception(frame[1], MODBUS_EX_ILLEGAL_DATA_ADDR);
      return;
    }
    u16_be_put(&out[3 + 2 * i], reg);
  }

  uint16_t resp_len_wo_crc = (uint16_t)(3 + 2 * qty);
  uint16_t crc = crc16_modbus(out, resp_len_wo_crc);
  out[resp_len_wo_crc + 0] = (uint8_t)(crc & 0xFF);
  out[resp_len_wo_crc + 1] = (uint8_t)((crc >> 8) & 0xFF);
  (void)HAL_UART_Transmit(s_huart, out, (uint16_t)(resp_len_wo_crc + 2), 200);
}

static void handle_write_single_reg(const uint8_t *frame, uint16_t len)
{
  (void)len;
  uint16_t addr = u16_be_get(&frame[2]);
  uint16_t val = u16_be_get(&frame[4]);

  if (!write_holding_reg(addr, val))
  {
    send_exception(frame[1], MODBUS_EX_ILLEGAL_DATA_ADDR);
    return;
  }
  (void)HAL_UART_Transmit(s_huart, (uint8_t *)frame, 8, 200);
}

static void process_frame(const uint8_t *frame, uint16_t len)
{
  if (len < 4)
  {
    return;
  }

  if (frame[0] != s_slave_id)
  {
    return;
  }

  if (len < 8)
  {
    return;
  }

  uint16_t crc_rx = (uint16_t)((uint16_t)frame[len - 1] << 8) | frame[len - 2];
  uint16_t crc_calc = crc16_modbus(frame, (uint16_t)(len - 2));
  if (crc_rx != crc_calc)
  {
    return;
  }

  uint8_t func = frame[1];
  switch (func)
  {
    case MODBUS_FUNC_READ_HOLDING_REGS:
      handle_read_holding_regs(frame, len);
      break;
    case MODBUS_FUNC_WRITE_SINGLE_REG:
      handle_write_single_reg(frame, len);
      break;
    default:
      send_exception(func, MODBUS_EX_ILLEGAL_FUNCTION);
      break;
  }
}

void ModbusRTUSlave_Init(UART_HandleTypeDef *huart, uint8_t slave_id)
{
  s_huart = huart;
  s_slave_id = (slave_id == 0) ? 1 : slave_id;
  s_rx_len = 0;
  s_last_rx_tick = HAL_GetTick();
  s_frame_ready = 0;
}

void ModbusRTUSlave_OnByte(uint8_t b)
{
  s_last_rx_tick = HAL_GetTick();

  if (s_frame_ready)
  {
    return;
  }

  if (s_rx_len < MODBUS_RTU_MAX_FRAME)
  {
    s_rx_buf[s_rx_len++] = b;
  }
  else
  {
    s_rx_len = 0;
  }
}

void ModbusRTUSlave_OnIdle(void)
{
  if (s_rx_len == 0)
  {
    return;
  }
  s_frame_ready = 1;
}

void ModbusRTUSlave_Poll(void)
{
  if (!s_frame_ready)
  {
    if (s_rx_len > 0)
    {
      uint32_t now = HAL_GetTick();
      if ((now - s_last_rx_tick) >= MODBUS_RTU_RX_SILENT_TIMEOUT_MS)
      {
        s_frame_ready = 1;
      }
    }
    else
    {
      return;
    }
  }

  uint16_t len = s_rx_len;
  uint8_t frame[MODBUS_RTU_MAX_FRAME];
  if (len > MODBUS_RTU_MAX_FRAME)
  {
    len = MODBUS_RTU_MAX_FRAME;
  }

  for (uint16_t i = 0; i < len; i++)
  {
    frame[i] = s_rx_buf[i];
  }

  s_rx_len = 0;
  s_frame_ready = 0;

  if (len >= 16U && ((len & 0x0007U) == 0U))
  {
    for (uint16_t off = 0; (uint16_t)(off + 8U) <= len; off = (uint16_t)(off + 8U))
    {
      process_frame(&frame[off], 8U);
    }
  }
  else
  {
    process_frame(frame, len);
  }
}

void ModbusRTUSlave_SetGasType(uint16_t gas_type)
{
  s_gas_type = gas_type;
}

void ModbusRTUSlave_SetConcentrationFloat(float conc)
{
  s_conc = conc;
}

uint16_t ModbusRTUSlave_GetGasType(void)
{
  return s_gas_type;
}

float ModbusRTUSlave_GetConcentrationFloat(void)
{
  return s_conc;
}
