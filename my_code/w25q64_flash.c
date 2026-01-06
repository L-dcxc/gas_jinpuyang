#include "w25q64_flash.h"

#include "main.h"
#include "spi.h"

#ifndef W25Q64_SPI_TIMEOUT_MS
#define W25Q64_SPI_TIMEOUT_MS 100U
#endif

#ifndef W25Q64_BUSY_TIMEOUT_MS
#define W25Q64_BUSY_TIMEOUT_MS 5000U
#endif

static void cs_low(void)
{
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
}

static void cs_high(void)
{
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

static int spi_tx(const uint8_t *tx, uint16_t len)
{
  return (HAL_SPI_Transmit(&hspi2, (uint8_t *)tx, len, W25Q64_SPI_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

static int spi_rx(uint8_t *rx, uint16_t len)
{
  return (HAL_SPI_Receive(&hspi2, rx, len, W25Q64_SPI_TIMEOUT_MS) == HAL_OK) ? 0 : -1;
}

static int read_status(uint8_t *sr)
{
  uint8_t cmd = 0x05;
  cs_low();
  if (spi_tx(&cmd, 1) != 0)
  {
    cs_high();
    return -1;
  }
  if (spi_rx(sr, 1) != 0)
  {
    cs_high();
    return -1;
  }
  cs_high();
  return 0;
}

static int wait_ready(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  for (;;)
  {
    uint8_t sr;
    if (read_status(&sr) != 0)
    {
      return -1;
    }
    if ((sr & 0x01U) == 0U)
    {
      return 0;
    }
    if ((HAL_GetTick() - start) > timeout_ms)
    {
      return -1;
    }
  }
}

static int write_enable(void)
{
  uint8_t cmd = 0x06;
  cs_low();
  if (spi_tx(&cmd, 1) != 0)
  {
    cs_high();
    return -1;
  }
  cs_high();
  return 0;
}

int W25Q64_Init(void)
{
  uint8_t cmd = 0x9F;
  uint8_t id[3] = {0};

  cs_high();
  cs_low();
  if (spi_tx(&cmd, 1) != 0)
  {
    cs_high();
    return -1;
  }
  if (spi_rx(id, 3) != 0)
  {
    cs_high();
    return -1;
  }
  cs_high();

  if (id[0] == 0x00U || id[0] == 0xFFU)
  {
    return -1;
  }
  return 0;
}

int W25Q64_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
  uint8_t cmd[4];
  if (buf == NULL || len == 0U)
  {
    return 0;
  }

  cmd[0] = 0x03;
  cmd[1] = (uint8_t)((addr >> 16) & 0xFFU);
  cmd[2] = (uint8_t)((addr >> 8) & 0xFFU);
  cmd[3] = (uint8_t)(addr & 0xFFU);

  cs_low();
  if (spi_tx(cmd, 4) != 0)
  {
    cs_high();
    return -1;
  }
  if (spi_rx(buf, (uint16_t)len) != 0)
  {
    cs_high();
    return -1;
  }
  cs_high();
  return 0;
}

int W25Q64_EraseSector4K(uint32_t addr)
{
  uint8_t cmd[4];

  if (wait_ready(W25Q64_BUSY_TIMEOUT_MS) != 0)
  {
    return -1;
  }
  if (write_enable() != 0)
  {
    return -1;
  }

  cmd[0] = 0x20;
  cmd[1] = (uint8_t)((addr >> 16) & 0xFFU);
  cmd[2] = (uint8_t)((addr >> 8) & 0xFFU);
  cmd[3] = (uint8_t)(addr & 0xFFU);

  cs_low();
  if (spi_tx(cmd, 4) != 0)
  {
    cs_high();
    return -1;
  }
  cs_high();

  return wait_ready(W25Q64_BUSY_TIMEOUT_MS);
}

static int page_program(uint32_t addr, const uint8_t *buf, uint16_t len)
{
  uint8_t cmd[4];

  if (len == 0U)
  {
    return 0;
  }

  if (wait_ready(W25Q64_BUSY_TIMEOUT_MS) != 0)
  {
    return -1;
  }
  if (write_enable() != 0)
  {
    return -1;
  }

  cmd[0] = 0x02;
  cmd[1] = (uint8_t)((addr >> 16) & 0xFFU);
  cmd[2] = (uint8_t)((addr >> 8) & 0xFFU);
  cmd[3] = (uint8_t)(addr & 0xFFU);

  cs_low();
  if (spi_tx(cmd, 4) != 0)
  {
    cs_high();
    return -1;
  }
  if (spi_tx(buf, len) != 0)
  {
    cs_high();
    return -1;
  }
  cs_high();

  return wait_ready(W25Q64_BUSY_TIMEOUT_MS);
}

int W25Q64_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
  uint32_t offset = 0;

  if (buf == NULL || len == 0U)
  {
    return 0;
  }

  while (offset < len)
  {
    uint32_t cur_addr = addr + offset;
    uint32_t page_rem = 256U - (cur_addr & 0xFFU);
    uint32_t chunk = (len - offset);
    if (chunk > page_rem)
    {
      chunk = page_rem;
    }

    if (page_program(cur_addr, &buf[offset], (uint16_t)chunk) != 0)
    {
      return -1;
    }
    offset += chunk;
  }

  return 0;
}
