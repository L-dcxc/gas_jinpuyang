#include "frn06.h"

#include "i2c.h"

#define FRN06_I2C_ADDR_7BIT  0x40
#define FRN06_I2C_ADDR_W     (FRN06_I2C_ADDR_7BIT << 1)

#define FRN06_CMD_FLOW       0x1000

#define FRN06_FLOW_RESP_LEN  5

#define FRN06_OFFSET_DEFAULT 30000L
#define FRN06_SCALE_DEFAULT  2500L

#ifndef FRN06_ZERO_DEADBAND_MSLM
#define FRN06_ZERO_DEADBAND_MSLM 10L
#endif

static int32_t s_frn06_offset = FRN06_OFFSET_DEFAULT;
static int32_t s_frn06_scale = FRN06_SCALE_DEFAULT;

static uint8_t frn06_crc8_poly131(const uint8_t *data, uint32_t len)
{
  uint8_t crc = 0x00;
  for (uint32_t i = 0; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
    {
      if (crc & 0x80)
      {
        crc = (uint8_t)((crc << 1) ^ 0x31);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

HAL_StatusTypeDef FRN06_Init(void)
{
  return HAL_OK;
}

HAL_StatusTypeDef FRN06_ReadParams(int32_t *offset, int32_t *scale)
{
  uint8_t cmd[2];
  uint8_t resp[41];
  uint8_t crc;
  HAL_StatusTypeDef st;

  if (offset == NULL || scale == NULL)
  {
    return HAL_ERROR;
  }

  cmd[0] = 0xCC;
  cmd[1] = 0xDD;

  st = HAL_I2C_Master_Transmit(&hi2c1, FRN06_I2C_ADDR_W, cmd, sizeof(cmd), 100);
  if (st != HAL_OK)
  {
    return st;
  }

  st = HAL_I2C_Master_Receive(&hi2c1, FRN06_I2C_ADDR_W, resp, sizeof(resp), 100);
  if (st != HAL_OK)
  {
    return st;
  }

  crc = frn06_crc8_poly131(resp, 40);
  if (crc != resp[40])
  {
    return HAL_ERROR;
  }

  {
    uint16_t off_u16 = (uint16_t)(((uint16_t)resp[8] << 8) | (uint16_t)resp[9]);
    uint16_t scale_u16 = (uint16_t)(((uint16_t)resp[10] << 8) | (uint16_t)resp[11]);
    *offset = (int32_t)off_u16;
    *scale = (int32_t)scale_u16;
    s_frn06_offset = *offset;
    s_frn06_scale = *scale;
    if (s_frn06_scale == 0)
    {
      s_frn06_scale = FRN06_SCALE_DEFAULT;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef FRN06_ReadFlowRaw(int32_t *raw)
{
  uint8_t cmd[2];
  uint8_t resp[FRN06_FLOW_RESP_LEN];
  uint8_t crc;
  HAL_StatusTypeDef st;

  if (raw == NULL)
  {
    return HAL_ERROR;
  }

  cmd[0] = (uint8_t)((FRN06_CMD_FLOW >> 8) & 0xFF);
  cmd[1] = (uint8_t)(FRN06_CMD_FLOW & 0xFF);

  st = HAL_I2C_Master_Transmit(&hi2c1, FRN06_I2C_ADDR_W, cmd, sizeof(cmd), 10);
  if (st != HAL_OK)
  {
    return st;
  }

  st = HAL_I2C_Master_Receive(&hi2c1, FRN06_I2C_ADDR_W, resp, sizeof(resp), 10);
  if (st != HAL_OK)
  {
    return st;
  }

  crc = frn06_crc8_poly131(resp, 4);
  if (crc != resp[4])
  {
    return HAL_ERROR;
  }

  {
    uint16_t u16 = (uint16_t)(((uint16_t)resp[0] << 8) | (uint16_t)resp[1]);
    *raw = (int32_t)u16;
  }

  return HAL_OK;
}

HAL_StatusTypeDef FRN06_ReadFlow_mslm(int32_t *flow_mslm)
{
  int32_t raw;
  HAL_StatusTypeDef st = FRN06_ReadFlowRaw(&raw);
  if (st != HAL_OK)
  {
    return st;
  }

  {
    int64_t num = (int64_t)(raw - s_frn06_offset) * 1000LL;
    int64_t den = (int64_t)s_frn06_scale;

    if (num >= 0)
    {
      num += den / 2;
    }
    else
    {
      num -= den / 2;
    }

    *flow_mslm = (int32_t)(num / den);
    if (*flow_mslm <= (int32_t)FRN06_ZERO_DEADBAND_MSLM && *flow_mslm >= (int32_t)(-FRN06_ZERO_DEADBAND_MSLM))
    {
      *flow_mslm = 0;
    }
  }

  return HAL_OK;
}
