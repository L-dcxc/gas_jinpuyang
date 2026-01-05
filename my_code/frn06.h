#ifndef __FRN06_H__
#define __FRN06_H__

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef FRN06_Init(void);
HAL_StatusTypeDef FRN06_ReadFlowRaw(int32_t *raw);
HAL_StatusTypeDef FRN06_ReadFlow_mslm(int32_t *flow_mslm);
HAL_StatusTypeDef FRN06_ReadParams(int32_t *offset, int32_t *scale);

#ifdef __cplusplus
}
#endif

#endif /* __FRN06_H__ */
