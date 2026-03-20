#include "AttitudeControllerMath.hpp"

#include <cmath>

namespace finware {
namespace app {

float clamp(float x, float lo, float hi) {
  if (x < lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

Mat3 transpose(const Mat3& a) {
  Mat3 out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      out.m[r][c] = a.m[c][r];
    }
  }
  return out;
}

Mat3 multiply(const Mat3& a, const Mat3& b) {
  Mat3 out{};
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      float acc = 0.0f;
      for (int k = 0; k < 3; ++k) {
        acc += a.m[r][k] * b.m[k][c];
      }
      out.m[r][c] = acc;
    }
  }
  return out;
}

Mat3 euler321ToDCM(float roll, float pitch, float yaw) {
  const float cr = std::cos(roll);
  const float sr = std::sin(roll);
  const float cp = std::cos(pitch);
  const float sp = std::sin(pitch);
  const float cy = std::cos(yaw);
  const float sy = std::sin(yaw);

  // Earth-to-body DCM for 3-2-1 Euler sequence.
  Mat3 c_be{};
  c_be.m[0][0] = cp * cy;
  c_be.m[0][1] = cp * sy;
  c_be.m[0][2] = -sp;

  c_be.m[1][0] = sr * sp * cy - cr * sy;
  c_be.m[1][1] = sr * sp * sy + cr * cy;
  c_be.m[1][2] = sr * cp;

  c_be.m[2][0] = cr * sp * cy + sr * sy;
  c_be.m[2][1] = cr * sp * sy - sr * cy;
  c_be.m[2][2] = cr * cp;

  return c_be;
}

Vec3 dcmErrorBody(const Mat3& current_be, const Mat3& cmd_be) {
  const Mat3 cerr = multiply(current_be, transpose(cmd_be));

  Vec3 e{};
  e.x = 0.5f * (cerr.m[2][1] - cerr.m[1][2]);
  e.y = 0.5f * (cerr.m[0][2] - cerr.m[2][0]);
  e.z = 0.5f * (cerr.m[1][0] - cerr.m[0][1]);
  return e;
}

}  // namespace app
}  // namespace finware
