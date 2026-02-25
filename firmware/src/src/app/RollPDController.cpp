#include "RollPDController.h"
#include <cmath>

float clamp(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float wrap_to_pi(float angle_rad) {
  // Wrap to [-pi, pi]
  while (angle_rad >  M_PI) angle_rad -= 2.0f * static_cast<float>(M_PI);
  while (angle_rad < -M_PI) angle_rad += 2.0f * static_cast<float>(M_PI);
  return angle_rad;
}

float apply_rate_limit(float u_cmd_rad,
                       float u_last_rad,
                       float dt_s,
                       float u_rate_max_rads) {
  // Max change allowed this step
  const float max_step = u_rate_max_rads * dt_s;

  const float du = u_cmd_rad - u_last_rad;
  if (du >  max_step) return u_last_rad + max_step;
  if (du < -max_step) return u_last_rad - max_step;
  return u_cmd_rad;
}

void roll_pd_reset(RollPDState& st, float u_init_rad) {
  st.initialized = true;
  st.u_last_rad  = u_init_rad;
}

float roll_pd_update(const RollPDParams& params,
                     RollPDState& st,
                     float phi_meas_rad,
                     float p_meas_rads,
                     float dt_s,
                     bool enable) {
  // Handle weird dt
  if (dt_s <= 0.0f) dt_s = 1e-3f;

  // Initialize on first run if user didn't reset explicitly
  if (!st.initialized) {
    st.initialized = true;
    st.u_last_rad  = 0.0f;
  }

  // If not enabled, command neutral but still rate-limit to avoid snapping
  if (!enable) {
    const float u_neutral = 0.0f;
    float u = apply_rate_limit(u_neutral, st.u_last_rad, dt_s, params.u_rate_max_rads);
    u = clamp(u, -params.u_max_rad, params.u_max_rad);
    st.u_last_rad = u;
    return u;
  }

  // Angle error (wrapped so you don't command the long way around)
  const float err_rad = wrap_to_pi(params.phi_cmd_rad - phi_meas_rad);

  // PD law (D uses gyro roll rate directly; no differentiating angle)
  float u_cmd = (params.kp * err_rad) - (params.kd * p_meas_rads);

  // Saturate command
  u_cmd = clamp(u_cmd, -params.u_max_rad, params.u_max_rad);

  // Rate limit (servo max speed)
  float u = apply_rate_limit(u_cmd, st.u_last_rad, dt_s, params.u_rate_max_rads);

  // Final clamp (safety)
  u = clamp(u, -params.u_max_rad, params.u_max_rad);

  st.u_last_rad = u;
  return u;
}
