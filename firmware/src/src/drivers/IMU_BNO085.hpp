#pragma once
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

namespace finware {

// Minimal POD for "latest sample"
struct IMU_Sample {
  uint64_t t_us;       // micros() when sample captured (time of last update below)
  float q[4];          // quaternion (w, x, y, z)
  uint8_t calib;       // 0..3 from sensorValue.status
  // NEW: raw acceleration (m/s^2) and turn rate (rad/s)
  float ax, ay, az;    // accelerometer (SH2_ACCELEROMETER)
  float gx, gy, gz;    // gyroscope (SH2_GYROSCOPE)
  uint32_t seq;        // increments on each fresh sensor event
};

// Optional Euler helper (degrees by convention here)
struct IMU_EulerDeg { float yaw, pitch, roll; };

class IMU_BNO085 {
public:
  IMU_BNO085(uint8_t csPin, uint8_t intPin, int8_t resetPin = 5);

  // report = SH2_ARVR_STABILIZED_RV (accurate) or SH2_GYRO_INTEGRATED_RV (fast)
  bool begin(sh2_SensorId_t report = SH2_ARVR_STABILIZED_RV, uint32_t reportIntervalUs = 5000);

  // Change rotation-vector report & interval (accel/gyro stay enabled at the same rate)
  bool setReport(sh2_SensorId_t report, uint32_t reportIntervalUs);

  // Poll at most one event and update the latest sample if it’s one we care about
  void tick();

  bool ok() const { return _healthy; }
  const IMU_Sample& latest() const { return _latest; }
  IMU_EulerDeg latestEulerDegrees() const;

  bool wasReset();
  uint32_t sequence() const;
  uint32_t quatSequence() const;
  uint8_t calibration() const;
  sh2_SensorId_t activeReport() const { return _reportType; }
  uint32_t activeIntervalUs() const { return _reportIntervalUs; }
  bool disableRotationVectorReport();

private:
  static void quatToEuler(float qw, float qx, float qy, float qz, IMU_EulerDeg& outDeg);
  bool enableReports_();  // enable RV + accel + gyro

  const uint8_t _cs, _int;
  const int8_t  _rst;
  Adafruit_BNO08x _bno;

  sh2_SensorId_t _reportType = SH2_ARVR_STABILIZED_RV;
  uint32_t _reportIntervalUs = 5000;

  sh2_SensorValue_t _sv{};
  IMU_Sample _latest{0, {1,0,0,0}, 0, 0,0,0, 0,0,0, 0};
  volatile uint32_t _seq = 0;
  volatile uint32_t _qseq = 0;
  bool _healthy = false;
};

} // namespace finware
