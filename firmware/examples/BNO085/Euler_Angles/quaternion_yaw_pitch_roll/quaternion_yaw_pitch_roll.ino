#include <Arduino.h>
#include <Adafruit_BNO08x.h>

#define BNO08X_CS    10
#define BNO08X_INT   9
#define BNO08X_RESET 5

static const uint32_t reportIntervalUs = 5000; // 200 Hz

Adafruit_BNO08x bno(BNO08X_RESET);
sh2_SensorValue_t sv;

// Latest accel values
float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;

uint32_t seq = 0;
uint32_t t_us = 0;

// // ---- Choose ONE accel report ----
// // 1) Raw accelerometer (includes gravity)  ✅ recommended for "raw accel"
// static const sh2_SensorId_t ACCEL_REPORT = SH2_ACCELEROMETER;

// 2) Linear acceleration (gravity removed)
static const sh2_SensorId_t ACCEL_REPORT = SH2_LINEAR_ACCELERATION;

bool enableReports() {
  // Only accelerometer
  return bno.enableReport(ACCEL_REPORT, (long)reportIntervalUs);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("BNO085 ACCEL ONLY TEST");

  if (!bno.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO085");
    while (1) delay(10);
  }

  if (!enableReports()) {
    Serial.println("Failed to enable accel report");
  }
}

void loop() {

  if (bno.wasReset()) {
    Serial.println("IMU reset detected");
    enableReports();
  }

  // Same single-event behavior as your finware code
  if (bno.getSensorEvent(&sv)) {

    if (sv.sensorId == ACCEL_REPORT) {
      // Adafruit uses sv.un.accelerometer for both ACCELEROMETER and LINEAR_ACCELERATION
      const auto &a = sv.un.accelerometer;

      ax = a.x;
      ay = a.y;
      az = a.z;

      t_us = micros();
      seq++;
    }
  }

  // Print at 50 Hz
  static uint32_t last_print = 0;
  if (millis() - last_print >= 20) {
    last_print += 20;

    Serial.print("seq="); Serial.print(seq);
    Serial.print(" t_us="); Serial.print(t_us);
    Serial.print(" | ax="); Serial.print(ax, 6);
    Serial.print(" ay="); Serial.print(ay, 6);
    Serial.print(" az="); Serial.println(az, 6);
  }
}
