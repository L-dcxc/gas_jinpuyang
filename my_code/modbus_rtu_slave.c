#include "modbus_rtu_slave.h"

#ifndef MODBUS_RTU_MAX_FRAME
#define MODBUS_RTU_MAX_FRAME 256
#endif

#ifndef MODBUS_RTU_RX_SILENT_TIMEOUT_MS
#define MODBUS_RTU_RX_SILENT_TIMEOUT_MS 30U
#endif

#define MODBUS_FUNC_READ_HOLDING_REGS 0x03
#define MODBUS_FUNC_WRITE_SINGLE_REG  0x06

#define MODBUS_EX_ILLEGAL_FUNCTION    0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDR   0x02
#define MODBUS_EX_ILLEGAL_DATA_VALUE  0x03

typedef struct
{
  UART_HandleTypeDef *huart;
  uint8_t slave_id;
  volatile uint8_t rx_buf[MODBUS_RTU_MAX_FRAME];
  volatile uint16_t rx_len;
  volatile uint32_t last_rx_tick;
  volatile uint8_t frame_ready;
} ModbusPort;

static ModbusPort s_ports[2];
static uint8_t s_port_count = 0;

static volatile uint16_t s_conc_u16[4] = {0, 0, 0, 0};
static volatile uint16_t s_pump_en = 0;
static volatile uint16_t s_leak_state = 0;
static volatile uint16_t s_zero_calib_req = 0xFFFFU;
static volatile uint16_t s_zero_calib_result = 0U;

volatile uint32_t g_modbus_rx_ok_frames = 0;
volatile uint32_t g_modbus_rx_bad_crc_frames = 0;

static ModbusPort *find_port(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return NULL;
  }
  for (uint8_t i = 0; i < s_port_count; i++)
  {
    if (s_ports[i].huart == huart)
    {
      return &s_ports[i];
    }
  }
  return NULL;
}

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

static void send_exception(ModbusPort *p, uint8_t func, uint8_t ex)
{
  uint8_t out[5];
  if (p == NULL || p->huart == NULL)
  {
    return;
  }
  out[0] = p->slave_id;
  out[1] = (uint8_t)(func | 0x80);
  out[2] = ex;
  uint16_t crc = crc16_modbus(out, 3);
  out[3] = (uint8_t)(crc & 0xFF);
  out[4] = (uint8_t)((crc >> 8) & 0xFF);
  (void)HAL_UART_Transmit(p->huart, out, sizeof(out), 100);
}

static int read_holding_reg(uint16_t addr_0based, uint16_t *out)
{
  if (addr_0based < 4U)
  {
    *out = (uint16_t)s_conc_u16[addr_0based];
    return 1;
  }
  if (addr_0based == 4U)
  {
    *out = (uint16_t)s_pump_en;
    return 1;
  }
  if (addr_0based == 5U)
  {
    *out = (uint16_t)s_leak_state;
    return 1;
  }
  if (addr_0based == 6U)
  {
    *out = (uint16_t)s_zero_calib_req;
    return 1;
  }
  if (addr_0based == 7U)
  {
    *out = (uint16_t)s_zero_calib_result;
    return 1;
  }
  return 0;
}

static int write_holding_reg(uint16_t addr_0based, uint16_t val)
{
  if (addr_0based == 4U)
  {
    if (val == 0U || val == 1U)
    {
      s_pump_en = val;
      return 1;
    }
    return 0;
  }
  if (addr_0based == 6U)
  {
    if (val == 0xFFFFU)
    {
      s_zero_calib_req = 0xFFFFU;
      return 1;
    }
    if (val < 4U)
    {
      s_zero_calib_req = val;
      s_zero_calib_result = 0U;
      return 1;
    }
    return 0;
  }
  return 0;
}

void ModbusRTUSlave_SetConcentrationU16(uint8_t ch, uint16_t conc)
{
  if (ch < 4U)
  {
    s_conc_u16[ch] = conc;
  }
}

uint16_t ModbusRTUSlave_GetConcentrationU16(uint8_t ch)
{
  if (ch < 4U)
  {
    return (uint16_t)s_conc_u16[ch];
  }
  return 0;
}

void ModbusRTUSlave_SetPumpEnable(uint16_t en)
{
  s_pump_en = (en != 0U) ? 1U : 0U;
}

uint16_t ModbusRTUSlave_GetPumpEnable(void)
{
  return (uint16_t)s_pump_en;
}

void ModbusRTUSlave_SetLeakState(uint16_t state)
{
  if (state > 2U)
  {
    state = 2U;
  }
  s_leak_state = state;
}

uint16_t ModbusRTUSlave_GetLeakState(void)
{
  return (uint16_t)s_leak_state;
}

uint16_t ModbusRTUSlave_GetZeroCalibReq(void)
{
  return (uint16_t)s_zero_calib_req;
}

void ModbusRTUSlave_ClearZeroCalibReq(void)
{
  s_zero_calib_req = 0xFFFFU;
}

void ModbusRTUSlave_SetZeroCalibResult(uint16_t result)
{
  s_zero_calib_result = result;
}

uint16_t ModbusRTUSlave_GetZeroCalibResult(void)
{
  return (uint16_t)s_zero_calib_result;
}

static void handle_read_holding_regs(ModbusPort *p, const uint8_t *frame, uint16_t len)
{
  (void)len;
  if (p == NULL || p->huart == NULL)
  {
    return;
  }
  uint16_t start = u16_be_get(&frame[2]);
  uint16_t qty = u16_be_get(&frame[4]);

  if (qty == 0 || qty > 125)
  {
    send_exception(p, frame[1], MODBUS_EX_ILLEGAL_DATA_VALUE);
    return;
  }

  uint8_t out[3 + 2 * 125 + 2];
  out[0] = p->slave_id;
  out[1] = MODBUS_FUNC_READ_HOLDING_REGS;
  out[2] = (uint8_t)(qty * 2);

  for (uint16_t i = 0; i < qty; i++)
  {
    uint16_t reg;
    if (!read_holding_reg((uint16_t)(start + i), &reg))
    {
      send_exception(p, frame[1], MODBUS_EX_ILLEGAL_DATA_ADDR);
      return;
    }
    u16_be_put(&out[3 + 2 * i], reg);
  }

  uint16_t resp_len_wo_crc = (uint16_t)(3 + 2 * qty);
  uint16_t crc = crc16_modbus(out, resp_len_wo_crc);
  out[resp_len_wo_crc + 0] = (uint8_t)(crc & 0xFF);
  out[resp_len_wo_crc + 1] = (uint8_t)((crc >> 8) & 0xFF);
  (void)HAL_UART_Transmit(p->huart, out, (uint16_t)(resp_len_wo_crc + 2), 200);
}

static void handle_write_single_reg(ModbusPort *p, const uint8_t *frame, uint16_t len)
{
  (void)len;
  if (p == NULL || p->huart == NULL)
  {
    return;
  }
  uint16_t addr = u16_be_get(&frame[2]);
  uint16_t val = u16_be_get(&frame[4]);

  if (!write_holding_reg(addr, val))
  {
    if (addr == 4U)
    {
      send_exception(p, frame[1], MODBUS_EX_ILLEGAL_DATA_VALUE);
    }
    else
    {
      send_exception(p, frame[1], MODBUS_EX_ILLEGAL_DATA_ADDR);
    }
    return;
  }
  (void)HAL_UART_Transmit(p->huart, (uint8_t *)frame, 8, 200);
}

static void process_frame(ModbusPort *p, const uint8_t *frame, uint16_t len)
{
  if (len < 4)
  {
    return;
  }

  if (p == NULL)
  {
    return;
  }

  if (frame[0] != p->slave_id)
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
    g_modbus_rx_bad_crc_frames++;
    return;
  }

  g_modbus_rx_ok_frames++;

  uint8_t func = frame[1];
  switch (func)
  {
    case MODBUS_FUNC_READ_HOLDING_REGS:
      handle_read_holding_regs(p, frame, len);
      break;
    case MODBUS_FUNC_WRITE_SINGLE_REG:
      handle_write_single_reg(p, frame, len);
      break;
    default:
      send_exception(p, func, MODBUS_EX_ILLEGAL_FUNCTION);
      break;
  }
}

void ModbusRTUSlave_Init(UART_HandleTypeDef *huart, uint8_t slave_id)
{
  ModbusPort *p = find_port(huart);
  if (p == NULL)
  {
    if (s_port_count >= (uint8_t)(sizeof(s_ports) / sizeof(s_ports[0])))
    {
      return;
    }
    p = &s_ports[s_port_count++];
    p->huart = huart;
    p->rx_len = 0;
    p->last_rx_tick = HAL_GetTick();
    p->frame_ready = 0;
  }
  p->slave_id = (slave_id == 0U) ? 1U : slave_id;
}

void ModbusRTUSlave_OnByteFromUart(UART_HandleTypeDef *huart, uint8_t b)
{
  ModbusPort *p = find_port(huart);
  if (p == NULL)
  {
    return;
  }
  p->last_rx_tick = HAL_GetTick();

  if (p->frame_ready)
  {
    return;
  }

  if (p->rx_len < MODBUS_RTU_MAX_FRAME)
  {
    p->rx_buf[p->rx_len++] = b;
  }
  else
  {
    p->rx_len = 0;
  }
}

void ModbusRTUSlave_OnIdleFromUart(UART_HandleTypeDef *huart)
{
  ModbusPort *p = find_port(huart);
  if (p == NULL)
  {
    return;
  }
  if (p->rx_len == 0)
  {
    return;
  }
  p->frame_ready = 1;
}

static void poll_one(ModbusPort *p)
{
  if (p == NULL)
  {
    return;
  }

  if (!p->frame_ready)
  {
    if (p->rx_len >= 8U)
    {
      uint32_t now = HAL_GetTick();
      if ((now - p->last_rx_tick) >= MODBUS_RTU_RX_SILENT_TIMEOUT_MS)
      {
        p->frame_ready = 1;
      }
      else
      {
        return;
      }
    }
    else
    {
      return;
    }
  }

  uint16_t len = p->rx_len;
  uint8_t frame[MODBUS_RTU_MAX_FRAME];
  if (len > MODBUS_RTU_MAX_FRAME)
  {
    len = MODBUS_RTU_MAX_FRAME;
  }

  for (uint16_t i = 0; i < len; i++)
  {
    frame[i] = p->rx_buf[i];
  }

  if (len < 8U)
  {
    p->frame_ready = 0;
    return;
  }

  p->rx_len = 0;
  p->frame_ready = 0;

  for (uint16_t off = 0; (uint16_t)(off + 8U) <= len;)
  {
    uint8_t addr = frame[off + 0U];
    uint8_t func = frame[off + 1U];
    if (addr == p->slave_id && (func == MODBUS_FUNC_READ_HOLDING_REGS || func == MODBUS_FUNC_WRITE_SINGLE_REG))
    {
      uint16_t crc_rx = (uint16_t)(((uint16_t)frame[off + 7U] << 8) | frame[off + 6U]);
      uint16_t crc_calc = crc16_modbus(&frame[off], 6U);
      if (crc_rx == crc_calc)
      {
        process_frame(p, &frame[off], 8U);
        off = (uint16_t)(off + 8U);
        continue;
      }
      else
      {
        g_modbus_rx_bad_crc_frames++;
      }
    }
    off++;
  }
}

void ModbusRTUSlave_PollAll(void)
{
  for (uint8_t i = 0; i < s_port_count; i++)
  {
    poll_one(&s_ports[i]);
  }
}
