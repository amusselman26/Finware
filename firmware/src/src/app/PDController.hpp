#pragma once

namespace finware {
namespace app {

class PDController {
 public:
  PDController(float kp, float kd);

  float update(float error, float body_rate) const;
  void setGains(float kp, float kd);

 private:
  float kp_;
  float kd_;
};

}  // namespace app
}  // namespace finware
