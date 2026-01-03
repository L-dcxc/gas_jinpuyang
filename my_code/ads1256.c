#include "ads1256.h"

#include <stdio.h>
#include "spi.h"
#include "gpio.h"

static void ads1256_cs_low(void);
static void ads1256_cs_high(void);
static uint8_t ads1256_spi_txrx(uint8_t data);
static void ads1256_short_delay(void);
static void ads1256_cmd(uint8_t cmd);
static void ads1256_write_reg(uint8_t reg, uint8_t value);
static uint8_t ads1256_read_reg(uint8_t reg);
static int32_t ads1256_sign_extend_24(uint32_t x);
static int ads1256_wait_drdy_low(uint32_t timeout_ms);

/* ADS1256 Commands */
#define ADS1256_CMD_WAKEUP   0x00
#define ADS1256_CMD_RDATA    0x01
#define ADS1256_CMD_RDATAC   0x03
#define ADS1256_CMD_SDATAC   0x0F
#define ADS1256_CMD_RREG     0x10
#define ADS1256_CMD_WREG     0x50
#define ADS1256_CMD_SELFCAL  0xF0
#define ADS1256_CMD_SYNC     0xFC
#define ADS1256_CMD_RESET    0xFE

/* ADS1256 Registers */
#define ADS1256_REG_STATUS   0x00
#define ADS1256_REG_MUX      0x01

#define ADS1256_REG_ADCON    0x02
#define ADS1256_REG_DRATE    0x03

/* MUX values */
#define ADS1256_MUXP_AIN0    0x00
#define ADS1256_MUXN_AINCOM  0x08

#define ADS1256_VREF_UV      2500000L
#define ADS1256_PGA          1
#define ADS1256_FS_CODE      8388607L

#define ADS1256_INPUT_SCALE_NUM 2
#define ADS1256_INPUT_SCALE_DEN 1

#define ADS1256_DRATE_CODE   0x23
#define ADS1256_DISCARD_SAMPLES 1
#define ADS1256_AVG_SAMPLES  1
#define ADS1256_USED_CHANNELS 4
#define ADS1256_MOVAVG_WIN   1

volatile int32_t g_ads1256_latest_raw[ADS1256_USED_CHANNELS] = {0};
volatile int32_t g_ads1256_latest_uv[ADS1256_USED_CHANNELS] = {0};

static int32_t ads1256_raw_to_uv(int32_t raw)
{
  int64_t num = (int64_t)raw * (int64_t)ADS1256_VREF_UV * (int64_t)ADS1256_INPUT_SCALE_NUM;
  int64_t den = (int64_t)ADS1256_FS_CODE * (int64_t)ADS1256_PGA * (int64_t)ADS1256_INPUT_SCALE_DEN;

  if (num >= 0)
  {
    num += den / 2;
  }
  else
  {
    num -= den / 2;
  }
  return (int32_t)(num / den);
}

static int32_t ads1256_read_raw_current_mux(void)
{
  uint8_t b0, b1, b2;
  uint32_t raw24;

  ads1256_cs_low();
  (void)ads1256_spi_txrx(ADS1256_CMD_SYNC);
  (void)ads1256_spi_txrx(ADS1256_CMD_WAKEUP);

  if (!ads1256_wait_drdy_low(300))
  {
    ads1256_cs_high();
    return (int32_t)0x80000000;
  }

  (void)ads1256_spi_txrx(ADS1256_CMD_RDATA);
  ads1256_short_delay();

  b0 = ads1256_spi_txrx(0xFF);
  b1 = ads1256_spi_txrx(0xFF);
  b2 = ads1256_spi_txrx(0xFF);
  ads1256_cs_high();

  raw24 = ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b2;
  return ads1256_sign_extend_24(raw24);
}

static int32_t ads1256_read_raw_ainx_aincom_filtered(uint8_t ch)
{
  uint8_t mux = (uint8_t)(((ch & 0x07) << 4) | ADS1256_MUXN_AINCOM);
  int64_t sum = 0;
  uint32_t ok = 0;
  int32_t value;

  ads1256_write_reg(ADS1256_REG_MUX, mux);

  for (uint32_t i = 0; i < (uint32_t)ADS1256_DISCARD_SAMPLES; i++)
  {
    (void)ads1256_read_raw_current_mux();
  }

  for (uint32_t i = 0; i < (uint32_t)ADS1256_AVG_SAMPLES; i++)
  {
    value = ads1256_read_raw_current_mux();
    if (value != (int32_t)0x80000000)
    {
      sum += (int64_t)value;
      ok++;
    }
  }

  if (ok == 0)
  {
    return (int32_t)0x80000000;
  }

  return (int32_t)(sum / (int64_t)ok);
}

void ADS1256_Update(void)
{
  static uint8_t inited = 0;
  static int32_t movavg_buf[ADS1256_USED_CHANNELS][ADS1256_MOVAVG_WIN] = {0};
  static int64_t movavg_sum[ADS1256_USED_CHANNELS] = {0};
  static uint8_t movavg_idx[ADS1256_USED_CHANNELS] = {0};
  static uint8_t movavg_cnt[ADS1256_USED_CHANNELS] = {0};

  if (!inited)
  {
    ADS1256_Init();
    inited = 1;
  }

  for (uint8_t ch = 0; ch < ADS1256_USED_CHANNELS; ch++)
  {
    int32_t raw = ads1256_read_raw_ainx_aincom_filtered(ch);
    if (raw == (int32_t)0x80000000)
    {
      g_ads1256_latest_raw[ch] = (int32_t)0x80000000;
      g_ads1256_latest_uv[ch] = (int32_t)0x80000000;
      continue;
    }

    {
      uint8_t idx = movavg_idx[ch];
      if (movavg_cnt[ch] < (uint8_t)ADS1256_MOVAVG_WIN)
      {
        movavg_cnt[ch]++;
      }
      else
      {
        movavg_sum[ch] -= (int64_t)movavg_buf[ch][idx];
      }
      movavg_buf[ch][idx] = raw;
      movavg_sum[ch] += (int64_t)raw;
      idx++;
      if (idx >= (uint8_t)ADS1256_MOVAVG_WIN)
      {
        idx = 0;
      }
      movavg_idx[ch] = idx;
      raw = (int32_t)(movavg_sum[ch] / (int64_t)movavg_cnt[ch]);
    }

    g_ads1256_latest_raw[ch] = raw;
    g_ads1256_latest_uv[ch] = ads1256_raw_to_uv(raw);
  }
}

int32_t ADS1256_GetLatestRaw(uint8_t ch)
{
  if (ch >= ADS1256_USED_CHANNELS)
  {
    return (int32_t)0x80000001;
  }
  return (int32_t)g_ads1256_latest_raw[ch];
}

int32_t ADS1256_GetLatestUv(uint8_t ch)
{
  if (ch >= ADS1256_USED_CHANNELS)
  {
    return (int32_t)0x80000001;
  }
  return (int32_t)g_ads1256_latest_uv[ch];
}

static void ads1256_cs_low(void)
{
  HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_RESET);
}

static void ads1256_cs_high(void)
{
  HAL_GPIO_WritePin(ADS_CS_GPIO_Port, ADS_CS_Pin, GPIO_PIN_SET);
}

static void ads1256_short_delay(void)
{
  for (volatile uint32_t i = 0; i < 200; i++)
  {
    __NOP();
  }
}

static uint8_t ads1256_spi_txrx(uint8_t data)
{
  uint8_t rx = 0;
  (void)HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 1000);
  return rx;
}

static void ads1256_cmd(uint8_t cmd)
{
  ads1256_cs_low();
  (void)ads1256_spi_txrx(cmd);
  ads1256_short_delay();
  ads1256_cs_high();
}

static void ads1256_write_reg(uint8_t reg, uint8_t value)
{
  ads1256_cs_low();
  (void)ads1256_spi_txrx((uint8_t)(ADS1256_CMD_WREG | (reg & 0x0F)));
  (void)ads1256_spi_txrx(0x00); /* write 1 register */
  (void)ads1256_spi_txrx(value);
  ads1256_short_delay();
  ads1256_cs_high();

  /* t11: delay after WREG (conservative) */
  HAL_Delay(2);
}

static uint8_t ads1256_read_reg(uint8_t reg)
{
  uint8_t val;

  ads1256_cs_low();
  (void)ads1256_spi_txrx((uint8_t)(ADS1256_CMD_RREG | (reg & 0x0F)));
  (void)ads1256_spi_txrx(0x00); /* read 1 register */
  HAL_Delay(1);
  val = ads1256_spi_txrx(0xFF);
  ads1256_cs_high();

  return val;
}

static int32_t ads1256_sign_extend_24(uint32_t x)
{
  if (x & 0x800000U)
  {
    return (int32_t)(x | 0xFF000000U);
  }
  return (int32_t)x;
}

static int ads1256_wait_drdy_low(uint32_t timeout_ms)
{
  uint32_t start = HAL_GetTick();
  while (HAL_GPIO_ReadPin(ADS_DRDY_GPIO_Port, ADS_DRDY_Pin) == GPIO_PIN_SET)
  {
    if ((HAL_GetTick() - start) > timeout_ms)
    {
      return 0;
    }
  }
  return 1;
}

void ADS1256_Init(void)
{
  /* Ensure CS high (idle), release reset */
  ads1256_cs_high();
  HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_SET);

  /* Hardware reset pulse */
  HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(2);
  HAL_GPIO_WritePin(ADS_RST_GPIO_Port, ADS_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(5);

  /* Send RESET command + stop continuous read mode */
  ads1256_cmd(ADS1256_CMD_RESET);
  HAL_Delay(5);
  ads1256_cmd(ADS1256_CMD_SDATAC);
  HAL_Delay(2);

  /* Wait for DRDY to become LOW (ready). If DRDY is floating/unwired this will timeout. */
  (void)ads1256_wait_drdy_low(1000);

  /* Align with vendor demo init sequence:
   * - STATUS=0x06 (MSB first, ACAL=1, BUFEN=1)
   * - ADCON=0x00 (PGA=1)
   * - DRATE=0x23 (10SPS)
   * Then SELFCAL while CS low.
   */
  ads1256_write_reg(ADS1256_REG_STATUS, 0x06);
  ads1256_write_reg(ADS1256_REG_ADCON, 0x00);
  ads1256_write_reg(ADS1256_REG_DRATE, ADS1256_DRATE_CODE);

  if (ads1256_wait_drdy_low(2000))
  {
    ads1256_cs_low();
    (void)ads1256_spi_txrx(ADS1256_CMD_SELFCAL);
    (void)ads1256_wait_drdy_low(3000);
    ads1256_cs_high();
  }
}

uint8_t ADS1256_ReadStatus(void)
{
  return ads1256_read_reg(ADS1256_REG_STATUS);
}

int32_t ADS1256_ReadOnce_AIN0_AINCOM(void)
{
  return ADS1256_ReadOnce_AINx_AINCOM(0);
}

int32_t ADS1256_ReadOnce_AINx_AINCOM(uint8_t ain)
{
  uint8_t mux;

  if (ain > 7)
  {
    return (int32_t)0x80000001; /* invalid channel flag */
  }

  mux = (uint8_t)(((ain & 0x07) << 4) | ADS1256_MUXN_AINCOM);

  /* Select AINx - AINCOM */
  ads1256_write_reg(ADS1256_REG_MUX, mux);

  return ads1256_read_raw_current_mux();
}

void ADS1256_SelfTest(void)
{
  uint8_t status;
  uint8_t id;
  GPIO_PinState drdy_state;

  ADS1256_Update();

  drdy_state = HAL_GPIO_ReadPin(ADS_DRDY_GPIO_Port, ADS_DRDY_Pin);
  printf("ADS1256 DRDY pin=%d (0=LOW ready, 1=HIGH not-ready/float)\r\n", (int)drdy_state);

  status = ADS1256_ReadStatus();
  id = (uint8_t)((status >> 4) & 0x0F);

  printf("ADS1256 STATUS=0x%02X, ID=0x%X\r\n", status, id);

  for (uint8_t ch = 0; ch < ADS1256_USED_CHANNELS; ch++)
  {
    int32_t uv = ADS1256_GetLatestUv(ch);
    int32_t abs_uv = (uv >= 0) ? uv : -uv;
    int32_t v_int = abs_uv / 1000000L;
    int32_t v_frac = abs_uv % 1000000L;
    printf("ADS1256 AIN%u-AINCOM raw=%ld, V=%c%ld.%06ld\r\n",
           ch,
           (long)ADS1256_GetLatestRaw(ch),
           (uv < 0) ? '-' : '+',
           (long)v_int,
           (long)v_frac);
  }
}
