#ifndef __FLOW_CTRL_H__
#define __FLOW_CTRL_H__

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef FlowCtrl_Init(void);
void FlowCtrl_SetTarget_mslm(int32_t target_mslm);
void FlowCtrl_SetTunings(int32_t kp, int32_t ki, int32_t kd);
void FlowCtrl_Reset(void);
void FlowCtrl_Update(void);

int32_t FlowCtrl_GetTarget_mslm(void);
int32_t FlowCtrl_GetMeasured_mslm(void);
uint32_t FlowCtrl_GetPwmCompare(void);

#ifdef __cplusplus
}
#endif

#endif /* __FLOW_CTRL_H__ */
