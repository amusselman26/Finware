#pragma once

#include <cstdint>

namespace finware {
namespace app {

struct Vec3 {
  float x;
  float y;
  float z;
};

struct Mat3 {
  float m[3][3];
};

struct FinCommands {
  float d1;
  float d2;
  float d3;
  float d4;
};

struct PIDGains {
  float kp;
  float kd;
};

struct AttitudeControllerConfig {
  PIDGains roll;
  PIDGains pitch;
  PIDGains yaw;
  float fin_limit_rad;
};

float clamp(float x, float lo, float hi);
Mat3 transpose(const Mat3& a);
Mat3 multiply(const Mat3& a, const Mat3& b);

// Frame convention:
// - "earth" frame is an inertial frame (e.g., NED/ENU chosen by the caller).
// - "body" frame is fixed to the rocket with +X forward, +Y right, +Z down/up per project convention.
// - Returned matrix C_be maps an earth-frame vector to body-frame coordinates:
//     v_b = C_be * v_e
// - Euler321 means yaw(psi) about earth Z, then pitch(theta), then roll(phi).
Mat3 euler321ToDCM(float roll, float pitch, float yaw);

// Attitude error in body frame from DCM skew-symmetric part.
// Computes:
//   C_err = C_current_be * C_cmd_be^T
//   e = 0.5 * vee(C_err - C_err^T)
// with:
//   e1 = 0.5 * (C_err(3,2) - C_err(2,3))
//   e2 = 0.5 * (C_err(1,3) - C_err(3,1))
//   e3 = 0.5 * (C_err(2,1) - C_err(1,2))
Vec3 dcmErrorBody(const Mat3& current_be, const Mat3& cmd_be);

}  // namespace app
}  // namespace finware
