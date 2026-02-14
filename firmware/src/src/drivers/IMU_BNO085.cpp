#include "IMU_BNO085.hpp"
using namespace finware;

IMU_BNO085::IMU_BNO085(uint8_t csPin, uint8_t intPin, int8_t resetPin)
: _cs(csPin), _int(intPin), _rst(resetPin), _bno(resetPin) {}

bool IMU_BNO085::begin(sh2_SensorId_t report, uint32_t reportIntervalUs) {
  _reportType = report;
  _reportIntervalUs = reportIntervalUs;

  if (!_bno.begin_SPI(_cs, _int)) {
    Serial.println("BNO085 SPI begin failed");
    _healthy = false;
    return false;
  }
  _healthy = true;
  return enableReports_();
}

bool IMU_BNO085::setReport(sh2_SensorId_t report, uint32_t reportIntervalUs) {
  _reportType = report;
  _reportIntervalUs = reportIntervalUs;
  if (!_healthy) return false;
  return enableReports_();
}

bool IMU_BNO085::enableReports_() {
  bool ok = true;
  // 1) Rotation vector (your selected flavor)
  ok &= _bno.enableReport(_reportType, 5000); // 5 ms = 200 Hz, which is the max rate for RV reports
  // 2) Linear acceleration (m/s^2, gravity removed by sensor)
  ok &= _bno.enableReport(SH2_LINEAR_ACCELERATION, 10000);  // 10 ms = 100 Hz, which is the max rate for accel/gyro reports
  // 3) Calibrated gyroscope (rad/s)
  ok &= _bno.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000); // 5 ms = 200 Hz, which is a reasonable rate for gyro data

  _healthy = _healthy && ok;
  return ok;
}

void IMU_BNO085::tick() {
  if (!_healthy) return;

  if (_bno.wasReset()) {
    enableReports_(); // re-enable streams after reset
  }

  if (_bno.getSensorEvent(&_sv)) {
    const uint32_t now_us = micros();

    switch (_sv.sensorId) {
      // --- Rotation vectors (quaternion) ---
      case SH2_ARVR_STABILIZED_RV: {
        if (_reportType == SH2_ARVR_STABILIZED_RV) {
          const auto &rv = _sv.un.arvrStabilizedRV;
          _latest.q[0] = rv.real;
          _latest.q[1] = rv.i;
          _latest.q[2] = rv.j;
          _latest.q[3] = rv.k;
          _latest.calib = _sv.status;
          _latest.t_us  = now_us;
          _latest.seq   = ++_seq;
        }
        break;
      }
      case SH2_GYRO_INTEGRATED_RV: {
        if (_reportType == SH2_GYRO_INTEGRATED_RV) {
          const auto &rv = _sv.un.gyroIntegratedRV;
          _latest.q[0] = rv.real;
          _latest.q[1] = rv.i;
          _latest.q[2] = rv.j;
          _latest.q[3] = rv.k;
          _latest.calib = _sv.status;
          _latest.t_us  = now_us;
          _latest.seq   = ++_seq;
        }
        break;
      }

      // --- Accelerometer (m/s^2) ---
      case SH2_LINEAR_ACCELERATION: {
        const auto &a = _sv.un.linearAcceleration; // m/s^2
        _latest.ax = a.x;
        _latest.ay = a.y;
        _latest.az = a.z;
        _latest.calib = _sv.status;
        _latest.t_us  = now_us;
        _latest.seq   = ++_seq;
        break;
      }

      // --- Gyroscope (rad/s), calibrated ---
      case SH2_GYROSCOPE_CALIBRATED: {
        const auto &g = _sv.un.gyroscope; // rad/s
        _latest.gx = g.x;
        _latest.gy = g.y;
        _latest.gz = g.z;
        _latest.calib = _sv.status;
        _latest.t_us  = now_us;
        _latest.seq   = ++_seq;
        break;
      }

      default:
        // ignore other reports
        break;
    }
  }
}


bool IMU_BNO085::wasReset() {
  if (!_healthy) return false;
  if (_bno.wasReset()) {
    enableReports_();
    return true;
  }
  return false;
}

uint32_t IMU_BNO085::sequence() const { return _seq; }
uint8_t IMU_BNO085::calibration() const { return _latest.calib; }

IMU_EulerDeg IMU_BNO085::latestEulerDegrees() const {
  IMU_EulerDeg out{};
  quatToEuler(_latest.q[0], _latest.q[1], _latest.q[2], _latest.q[3], out);
  return out;
}

// Quaternion -> Euler (degrees)
void IMU_BNO085::quatToEuler(float qr, float qi, float qj, float qk, IMU_EulerDeg& outDeg) {
  const float sqr = qr*qr, sqi = qi*qi, sqj = qj*qj, sqk = qk*qk;
  const float yaw   = atan2f(2.f * (qi*qj + qk*qr), (sqi - sqj - sqk + sqr));
  const float pitch = asinf (-2.f * (qi*qk - qj*qr) / (sqi + sqj + sqk + sqr));
  const float roll  = atan2f(2.f * (qj*qk + qi*qr), (-sqi - sqj + sqk + sqr));
  outDeg.yaw = yaw * RAD_TO_DEG; outDeg.pitch = pitch * RAD_TO_DEG; outDeg.roll = roll * RAD_TO_DEG;
}
