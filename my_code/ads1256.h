#ifndef __ADS1256_H__
#define __ADS1256_H__

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void ADS1256_Init(void);
uint8_t ADS1256_ReadStatus(void);
int32_t ADS1256_ReadOnce_AINx_AINCOM(uint8_t ain);
int32_t ADS1256_ReadOnce_AIN0_AINCOM(void);
void ADS1256_SelfTest(void);

#define ADS1256_CH_COUNT 4

extern volatile int32_t g_ads1256_latest_raw[ADS1256_CH_COUNT];
extern volatile int32_t g_ads1256_latest_uv[ADS1256_CH_COUNT];

void ADS1256_Update(void);
int32_t ADS1256_GetLatestRaw(uint8_t ch);
int32_t ADS1256_GetLatestUv(uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* __ADS1256_H__ */
