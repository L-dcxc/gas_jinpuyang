#include "gas_calib.h"

#include "w25q64_flash.h"

#include <stddef.h>

#ifndef GAS_CALIB_FLASH_BASE
#define GAS_CALIB_FLASH_BASE 0x7FF000UL
#endif

#define GAS_CALIB_MAGIC 0x47435A50UL
#define GAS_CALIB_VER   1UL

typedef struct
{
  uint32_t magic;
  uint32_t version;
  int32_t zero_uv[GAS_CALIB_CH_COUNT];
  uint32_t crc32;
} GasCalibBlob;

static uint32_t crc32_calc(const uint8_t *data, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFFUL;
  for (uint32_t i = 0; i < len; i++)
  {
    crc ^= (uint32_t)data[i];
    for (uint8_t b = 0; b < 8U; b++)
    {
      uint32_t mask = (uint32_t)-(int32_t)(crc & 1UL);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

static int blob_is_valid(const GasCalibBlob *b)
{
  if (b == NULL)
  {
    return 0;
  }

  if (b->magic != GAS_CALIB_MAGIC || b->version != GAS_CALIB_VER)
  {
    return 0;
  }

  {
    uint32_t calc = crc32_calc((const uint8_t *)b, (uint32_t)(sizeof(GasCalibBlob) - sizeof(uint32_t)));
    if (calc != b->crc32)
    {
      return 0;
    }
  }

  for (uint8_t i = 0; i < GAS_CALIB_CH_COUNT; i++)
  {
    int32_t uv = b->zero_uv[i];
    if (uv < 0 || uv > 3000000)
    {
      return 0;
    }
  }

  return 1;
}

int GasCalib_LoadZeroUv(int32_t zero_uv_out[GAS_CALIB_CH_COUNT])
{
  GasCalibBlob b;

  if (zero_uv_out == NULL)
  {
    return -1;
  }

  if (W25Q64_Read(GAS_CALIB_FLASH_BASE, (uint8_t *)&b, (uint32_t)sizeof(b)) != 0)
  {
    return -1;
  }

  if (!blob_is_valid(&b))
  {
    return -1;
  }

  for (uint8_t i = 0; i < GAS_CALIB_CH_COUNT; i++)
  {
    zero_uv_out[i] = b.zero_uv[i];
  }
  return 0;
}

int GasCalib_SaveZeroUv(const int32_t zero_uv[GAS_CALIB_CH_COUNT])
{
  GasCalibBlob b;

  if (zero_uv == NULL)
  {
    return -1;
  }

  b.magic = GAS_CALIB_MAGIC;
  b.version = GAS_CALIB_VER;
  for (uint8_t i = 0; i < GAS_CALIB_CH_COUNT; i++)
  {
    b.zero_uv[i] = zero_uv[i];
  }
  b.crc32 = crc32_calc((const uint8_t *)&b, (uint32_t)(sizeof(GasCalibBlob) - sizeof(uint32_t)));

  if (W25Q64_EraseSector4K(GAS_CALIB_FLASH_BASE) != 0)
  {
    return -1;
  }

  if (W25Q64_Write(GAS_CALIB_FLASH_BASE, (const uint8_t *)&b, (uint32_t)sizeof(b)) != 0)
  {
    return -1;
  }

  return 0;
}
