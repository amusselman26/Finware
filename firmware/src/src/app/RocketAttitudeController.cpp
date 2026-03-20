#include "RocketAttitudeController.hpp"

namespace finware {
namespace app {

RocketAttitudeController::RocketAttitudeController(const AttitudeControllerConfig& config)
    : roll_pd_(config.roll.kp, config.roll.kd),
      pitch_pd_(config.pitch.kp, config.pitch.kd),
      yaw_pd_(config.yaw.kp, config.yaw.kd),
      mixer_(config.fin_limit_rad) {}

Vec3 RocketAttitudeController::computeError(const Mat3& current_dcm_be,
                                            const Vec3& cmd_euler_rpy) const {
  const Mat3 cmd_dcm_be =
      euler321ToDCM(cmd_euler_rpy.x, cmd_euler_rpy.y, cmd_euler_rpy.z);
  return dcmErrorBody(current_dcm_be, cmd_dcm_be);
}

FinCommands RocketAttitudeController::update(const Mat3& current_dcm_be,
                                             const Vec3& cmd_euler_rpy,
                                             const Vec3& body_rates_pqr) const {
  const Vec3 e_b = computeError(current_dcm_be, cmd_euler_rpy);

  const float u_roll = roll_pd_.update(e_b.x, body_rates_pqr.x);
  const float u_pitch = pitch_pd_.update(e_b.y, body_rates_pqr.y);
  const float u_yaw = yaw_pd_.update(e_b.z, body_rates_pqr.z);

  return mixer_.mix(u_roll, u_pitch, u_yaw);
}

}  // namespace app
}  // namespace finware
