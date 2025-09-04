#include "IMU_BNO085.hpp"


IMU_BNO085::IMU_BNO085(uint8_t csPin, uint8_t intPin, int8_t resetPin)
: _cs(csPin), _int(intPin), _rst(resetPin), _bno(resetPin) {}

bool IMU_BNO085::begin(sh2_SensorId_t report, uint32_t reportIntervalUs) {
  _reportType = report;
  _reportIntervalUs = reportIntervalUs;

  // Note: Adafruit_BNO08x::begin_SPI(cs, intPin) ignores reset if given in ctor.
  if (!_bno.begin_SPI(_cs, _int)) {
    _healthy = false;
    return false;
  }
  _healthy = true;

  // Select the desired report stream
  return enableReport_();
}

bool IMU_BNO085::setReport(sh2_SensorId_t report, uint32_t reportIntervalUs) {
  _reportType = report;
  _reportIntervalUs = reportIntervalUs;
  if (!_healthy) return false;
  return enableReport_();
}

bool IMU_BNO085::enableReport_() {
  // Request rotation-vector style report at the configured period
  // SH2_ARVR_STABILIZED_RV ~ accurate @ ~250 Hz max; SH2_GYRO_INTEGRATED_RV ~ up to ~1 kHz
  bool ok = _bno.enableReport(_reportType, (long)_reportIntervalUs);
  _healthy = _healthy && ok;
  return ok;
}

void IMU_BNO085::tick() {
  if (!_healthy) return;

  // If the chip reports a reset, re-enable our report
  if (_bno.wasReset()) {
    // keep running but refresh reports
    enableReport_();
  }

  // Pull at most one event; if it matches our report, cache latest quaternion
  if (_bno.getSensorEvent(&_sv)) {
    switch (_sv.sensorId) {
      case SH2_ARVR_STABILIZED_RV: {
        if (_reportType != SH2_ARVR_STABILIZED_RV) break;
        const auto &rv = _sv.un.arvrStabilizedRV;
        _latest.t_us = finware::Clock::now();
        _latest.q[0] = rv.real; _latest.q[1] = rv.i; _latest.q[2] = rv.j; _latest.q[3] = rv.k;
        _latest.calib = _sv.status;           // 0..3
        _latest.seq = ++_seq;
        break;
      }
      case SH2_GYRO_INTEGRATED_RV: {
        if (_reportType != SH2_GYRO_INTEGRATED_RV) break;
        const auto &rv = _sv.un.gyroIntegratedRV;
        _latest.t_us = finware::Clock::now();
        _latest.q[0] = rv.real; _latest.q[1] = rv.i; _latest.q[2] = rv.j; _latest.q[3] = rv.k;
        _latest.calib = _sv.status;
        _latest.seq = ++_seq;
        break;
      }
      default:
        // Other reports ignored in this wrapper
        break;
    }
  }
}

bool IMU_BNO085::wasReset() {
  if (!_healthy) return false;
  if (_bno.wasReset()) {
    enableReport_();
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

// Quaternion -> Euler (degrees), same math as your demo
void IMU_BNO085::quatToEuler(float qr, float qi, float qj, float qk, IMU_EulerDeg& outDeg) {
  const float sqr = qr*qr;
  const float sqi = qi*qi;
  const float sqj = qj*qj;
  const float sqk = qk*qk;

  float yaw   = atan2f(2.0f * (qi*qj + qk*qr), (sqi - sqj - sqk + sqr));
  float pitch = asinf (-2.0f * (qi*qk - qj*qr) / (sqi + sqj + sqk + sqr));
  float roll  = atan2f(2.0f * (qj*qk + qi*qr), (-sqi - sqj + sqk + sqr));

  outDeg.yaw   = yaw   * RAD_TO_DEG;
  outDeg.pitch = pitch * RAD_TO_DEG;
  outDeg.roll  = roll  * RAD_TO_DEG;
}
