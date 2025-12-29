#include "pid.h"

static int32_t clamp_i32(int32_t x, int32_t lo, int32_t hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void PID_Init(PID_Controller *pid, int32_t kp, int32_t ki, int32_t kd, int32_t out_min, int32_t out_max)
{
  if (pid == 0)
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->out_min = out_min;
  pid->out_max = out_max;

  pid->i_state = 0;
  pid->prev_err = 0;

  pid->i_min = out_min;
  pid->i_max = out_max;
}

void PID_SetIntegralLimit(PID_Controller *pid, int32_t i_min, int32_t i_max)
{
  if (pid == 0)
  {
    return;
  }
  pid->i_min = i_min;
  pid->i_max = i_max;
  pid->i_state = clamp_i32(pid->i_state, pid->i_min, pid->i_max);
}

void PID_Reset(PID_Controller *pid)
{
  if (pid == 0)
  {
    return;
  }
  pid->i_state = 0;
  pid->prev_err = 0;
}

int32_t PID_Update(PID_Controller *pid, int32_t setpoint, int32_t measurement)
{
  int32_t err;
  int32_t p;
  int32_t d;
  int32_t out;

  if (pid == 0)
  {
    return 0;
  }

  err = setpoint - measurement;

  p = (int32_t)((int64_t)pid->kp * (int64_t)err / 1000LL);

  pid->i_state += (int32_t)((int64_t)pid->ki * (int64_t)err / 1000LL);
  pid->i_state = clamp_i32(pid->i_state, pid->i_min, pid->i_max);

  d = (int32_t)((int64_t)pid->kd * (int64_t)(err - pid->prev_err) / 1000LL);

  out = p + pid->i_state + d;
  out = clamp_i32(out, pid->out_min, pid->out_max);

  pid->prev_err = err;
  return out;
}
