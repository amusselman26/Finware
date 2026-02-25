#pragma once
#include <cstdint>

// Parameters you tune
struct RollPDParams {
  float kp;           // proportional gain
  float kd;           // derivative gain (uses gyro roll rate p)
  float phi_cmd_rad;  // desired roll angle [rad]

  float u_max_rad;        // max fin deflection [rad]
  float u_rate_max_rads;  // max servo speed [rad/s]
};

// State that must persist across loops (for rate limiter)
struct RollPDState {
  bool  initialized;
  float u_last_rad;   // last output command [rad]
};

// --- Helpers ---
float clamp(float x, float lo, float hi);
float wrap_to_pi(float angle_rad);

// Rate limiter: limits how quickly command can change
float apply_rate_limit(float u_cmd_rad,
                       float u_last_rad,
                       float dt_s,
                       float u_rate_max_rads);

// Reset controller state (call at boot and/or state transitions)
void roll_pd_reset(RollPDState& st, float u_init_rad = 0.0f);

// Main update (call every loop)
// Inputs:
//   phi_meas_rad: measured/estimated roll angle [rad]
//   p_meas_rads:  measured roll rate (gyro) [rad/s]
//   dt_s:         timestep [s]
//   enable:       true during Launch->Apogee (or whenever you want control)
// Output:
//   fin command [rad]
float roll_pd_update(const RollPDParams& params,
                     RollPDState& st,
                     float phi_meas_rad,
                     float p_meas_rads,
                     float dt_s,
                     bool enable);
