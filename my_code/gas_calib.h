#ifndef GAS_CALIB_H
#define GAS_CALIB_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GAS_CALIB_CH_COUNT 4

int GasCalib_LoadZeroUv(int32_t zero_uv_out[GAS_CALIB_CH_COUNT]);
int GasCalib_SaveZeroUv(const int32_t zero_uv[GAS_CALIB_CH_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
