#pragma once

#include "AttitudeControllerMath.hpp"
#include "FinMixer.hpp"
#include "PDController.hpp"

namespace finware {
namespace app {

class RocketAttitudeController {
 public:
  explicit RocketAttitudeController(const AttitudeControllerConfig& config);

  // Full control update:
  // 1) Build commanded C_be from commanded roll/pitch/yaw.
  // 2) Compute body-frame attitude error from DCMs.
  // 3) Run axis PD controllers.
  // 4) Mix axis commands to fin deflections.
  FinCommands update(const Mat3& current_dcm_be,
                     const Vec3& cmd_euler_rpy,
                     const Vec3& body_rates_pqr) const;

  // Computes body-frame attitude error e_b between current and commanded attitude.
  Vec3 computeError(const Mat3& current_dcm_be,
                    const Vec3& cmd_euler_rpy) const;

 private:
  PDController roll_pd_;
  PDController pitch_pd_;
  PDController yaw_pd_;
  FinMixer mixer_;
};

}  // namespace app
}  // namespace finware
