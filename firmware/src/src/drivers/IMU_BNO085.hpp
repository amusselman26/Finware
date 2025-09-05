#pragma once
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#include "platform/Clock.hpp"

namespace finware {
// Minimal POD for "latest sample"
struct IMU_Sample {
  uint64_t t_us;       // micros() when sample captured
  float q[4];          // quaternion (w, x, y, z)
  uint8_t calib;       // 0..3 from sensorValue.status
  uint32_t seq;        // increments on each fresh sample
};

// Optional Euler helper (degrees by convention here)
struct IMU_EulerDeg {
  float yaw;
  float pitch;
  float roll;
};

class IMU_BNO085 {
public:
  // Preferred: SPI bring-up (matches your demo)
  // csPin: chip select, intPin: data-ready/interrupt, resetPin: sensor reset
  IMU_BNO085(uint8_t csPin, uint8_t intPin, int8_t resetPin = -1);

  // Begin and select the rotation-vector report & rate (µs interval)
  // report = SH2_ARVR_STABILIZED_RV (accurate) or SH2_GYRO_INTEGRATED_RV (fast)
  bool begin(sh2_SensorId_t report = SH2_ARVR_STABILIZED_RV, uint32_t reportIntervalUs = 5000);

  // Change report at runtime (non-blocking)
  bool setReport(sh2_SensorId_t report, uint32_t reportIntervalUs);

  // Non-blocking: read at most one sensor event; update latest if it matches the chosen report
  void tick();

  // True if at least one sample has been received recently
  bool ok() const { return _healthy; }

  // Access the most recent quaternion sample (thread-safe enough for Arduino single core)
  const IMU_Sample& latest() const { return _latest; }

  // Convenience: get Euler angles in degrees computed from the latest quaternion
  IMU_EulerDeg latestEulerDegrees() const;

  // Diagnostics
  bool wasReset();                 // checks chip-reset flag; re-enables report if needed
  uint32_t sequence() const;       // monotonically increasing sequence number
  uint8_t calibration() const;     // 0..3 (3 = best)
  sh2_SensorId_t activeReport() const { return _reportType; }
  uint32_t activeIntervalUs() const { return _reportIntervalUs; }

private:
  // Quaternion->Euler math (degrees if deg=true)
  static void quatToEuler(float qw, float qx, float qy, float qz, IMU_EulerDeg& outDeg);

  bool enableReport_();

  // Pins / HAL
  const uint8_t _cs;
  const uint8_t _int;
  const int8_t  _rst;

  // Underlying Adafruit driver
  Adafruit_BNO08x _bno;

  // Config
  sh2_SensorId_t _reportType = SH2_ARVR_STABILIZED_RV;
  uint32_t _reportIntervalUs = 5000;

  // State
  sh2_SensorValue_t _sv{};
  IMU_Sample _latest{0, {1,0,0,0}, 0, 0};
  volatile uint32_t _seq = 0;
  bool _healthy = false;
};
} // namespace finware