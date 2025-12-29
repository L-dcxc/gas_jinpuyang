#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  int32_t kp;
  int32_t ki;
  int32_t kd;
  int32_t out_min;
  int32_t out_max;
  int32_t i_state;
  int32_t prev_err;
  int32_t i_min;
  int32_t i_max;
} PID_Controller;

void PID_Init(PID_Controller *pid, int32_t kp, int32_t ki, int32_t kd, int32_t out_min, int32_t out_max);
void PID_SetIntegralLimit(PID_Controller *pid, int32_t i_min, int32_t i_max);
void PID_Reset(PID_Controller *pid);
int32_t PID_Update(PID_Controller *pid, int32_t setpoint, int32_t measurement);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
