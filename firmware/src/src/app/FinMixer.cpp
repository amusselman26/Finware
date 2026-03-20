#include "FinMixer.hpp"

#include <cmath>

namespace finware {
namespace app {

FinMixer::FinMixer(float fin_limit_rad) : fin_limit_rad_(fin_limit_rad) {}

FinCommands FinMixer::mix(float u_roll, float u_pitch, float u_yaw) const {
  FinCommands out{};

  // Mixer assumptions:
  //   My ~ (d2 - d4)
  //   Mz ~ (d1 - d3)
  //   Mx ~ -(d1 + d2 + d3 + d4)
  out.d1 = (0.5f * u_yaw) - (0.25f * u_roll);
  out.d2 = (-0.5f * u_pitch) - (0.25f * u_roll);
  out.d3 = (-0.5f * u_yaw) - (0.25f * u_roll);
  out.d4 = (0.5f * u_pitch) - (0.25f * u_roll);

  const float max_abs =
      fmaxf(fmaxf(fabsf(out.d1), fabsf(out.d2)), fmaxf(fabsf(out.d3), fabsf(out.d4)));

  // Uniform desaturation keeps command direction while fitting all fins into limits.
  if (max_abs > fin_limit_rad_ && max_abs > 0.0f) {
    const float scale = fin_limit_rad_ / max_abs;
    out.d1 *= scale;
    out.d2 *= scale;
    out.d3 *= scale;
    out.d4 *= scale;
  }

  // Final hard clamp for safety.
  out.d1 = clamp(out.d1, -fin_limit_rad_, fin_limit_rad_);
  out.d2 = clamp(out.d2, -fin_limit_rad_, fin_limit_rad_);
  out.d3 = clamp(out.d3, -fin_limit_rad_, fin_limit_rad_);
  out.d4 = clamp(out.d4, -fin_limit_rad_, fin_limit_rad_);

  return out;
}

void FinMixer::setFinLimit(float fin_limit_rad) {
  fin_limit_rad_ = fin_limit_rad;
}

float FinMixer::finLimit() const {
  return fin_limit_rad_;
}

}  // namespace app
}  // namespace finware
