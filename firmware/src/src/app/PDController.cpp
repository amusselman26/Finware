#include "PDController.hpp"

namespace finware {
namespace app {

PDController::PDController(float kp, float kd) : kp_(kp), kd_(kd) {}

float PDController::update(float error, float body_rate) const {
  // u = -kp*error - kd*body_rate
  return -(kp_ * error) - (kd_ * body_rate);
}

void PDController::setGains(float kp, float kd) {
  kp_ = kp;
  kd_ = kd;
}

}  // namespace app
}  // namespace finware
