#pragma once

#include "AttitudeControllerMath.hpp"

namespace finware {
namespace app {

class FinMixer {
 public:
  explicit FinMixer(float fin_limit_rad);

  FinCommands mix(float u_roll, float u_pitch, float u_yaw) const;

  void setFinLimit(float fin_limit_rad);
  float finLimit() const;

 private:
  float fin_limit_rad_;
};

}  // namespace app
}  // namespace finware
