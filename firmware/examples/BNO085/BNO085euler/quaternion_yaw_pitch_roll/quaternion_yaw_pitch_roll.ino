#include <Arduino.h>
#include <Adafruit_BNO08x.h>

// SPI pins
#define BNO08X_CS    10
#define BNO08X_INT   9
#define BNO08X_RESET 5

// #define FAST_MODE

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable report");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("Adafruit BNO08x quaternion print");

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) delay(10);
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportIntervalUs);

  Serial.println("dt_us\tstatus\tsensorId\tqw\tqx\tqy\tqz");
  delay(100);
}

uint32_t last_print_time = 0;

void loop() {
  if (bno08x.wasReset()) {
    Serial.println("sensor was reset");
    setReports(reportType, reportIntervalUs);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    static uint32_t last = 0;
    uint32_t now = micros();
    uint32_t dt = now - last;
    last = now;

    float qw=0, qx=0, qy=0, qz=0;

    switch (sensorValue.sensorId) {
      case SH2_ARVR_STABILIZED_RV: {
        const auto &rv = sensorValue.un.arvrStabilizedRV;
        qw = rv.real; qx = rv.i; qy = rv.j; qz = rv.k;
        break; // IMPORTANT: prevent fall-through
      }
      case SH2_GYRO_INTEGRATED_RV: {
        const auto &gi = sensorValue.un.gyroIntegratedRV;
        qw = gi.real; qx = gi.i; qy = gi.j; qz = gi.k;
        break;
      }
      default:
        return; // ignore other reports
    }

    if (now - last_print_time > 100000) {
      Serial.print(now); Serial.print(",");
      Serial.print(qw, 3); Serial.print(",");
      Serial.print(qx, 3); Serial.print(",");
      Serial.print(qy, 3); Serial.print(",");
      Serial.println(qz, 3);
      last_print_time = now;
    }
  }
}