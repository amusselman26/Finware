#include <Arduino.h>
#include <Wire.h>
#include "drivers/IMU_BNO085.hpp"
#include "drivers/GNSS_UBX.hpp"
#include "app/eskf15.h"

using namespace finware;

// IMU instance (CS=10, INT=9, RST=5)
IMU_BNO085 imu(10, 9, 5);
GNSS_UBX gnss(0x42);
eskf::ESKF15 kf;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 100; // print at most every 1000ms
unsigned long loopRateWindowStartMs = 0;
uint32_t loopCount = 0;

uint32_t prev_seq = 0;
uint32_t prev_gnss_seq = 0;
unsigned long prev_time_us = 0;

bool gpsRefSet = false;
float refLatRad = 0.0f;
float refLonRad = 0.0f;
float refAltM = 0.0f;

constexpr float EARTH_RADIUS_M = 6378137.0f;
constexpr float GPS_SIGMA_POS_M = 3.0f;
constexpr float GPS_SIGMA_VEL_MPS = 1.0f;
constexpr float GPS_HEADING_MIN_SPEED_MPS = 3.0f;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();

  Serial.println("Starting ESKF predict...");

  // Request calibrated gyroscope (accel & gyro enabled at 5ms = 200 Hz)
  if (!imu.begin(SH2_GYROSCOPE_CALIBRATED, 5000)) {
    Serial.println("Failed to initialize IMU!");
    while (1) delay(100);
  }

  // Prime once so latest() is valid
  imu.tick();
  prev_seq = imu.latest().seq;
  prev_time_us = imu.latest().t_us;

  if (!gnss.begin(10)) {
    Serial.println("Failed to initialize GNSS!");
  }

  gnss.tick();
  prev_gnss_seq = gnss.latest().seq;

  Serial.println("IMU + ESKF initialized.");
  Serial.println("t_us,qw,qx,qy,qz");

  loopRateWindowStartMs = millis();
}

void loop() {
  loopCount++;
  const unsigned long nowMs = millis();
  if (nowMs - loopRateWindowStartMs >= 1000) {
    Serial.print("loops_per_sec,");
    Serial.println(loopCount);
    loopCount = 0;
    loopRateWindowStartMs = nowMs;
  }

  imu.tick();

  const IMU_Sample& s = imu.latest();

  // Only act when a NEW sample arrives
  if (s.seq == prev_seq) return;
  prev_seq = s.seq;

  unsigned long now_us = s.t_us;
  float dt_s = (now_us - prev_time_us) / 1e6f;
  prev_time_us = now_us;

  // ESKF predict step with IMU gyro+accel
  if (dt_s > 0.0f) {
    kf.predict(dt_s,
               eskf::Vec3(s.gx, s.gy, s.gz),
               eskf::Vec3(s.ax, s.ay, s.az));
  }

  gnss.tick();
  const GNSS_Sample& g = gnss.latest();

  // Heading is observable only with a trusted heading cue (GPS course while moving).
  kf.heading_observable = (g.sats_used >= 4) && (g.speed_mps > GPS_HEADING_MIN_SPEED_MPS);

  if (g.seq != prev_gnss_seq) {
    prev_gnss_seq = g.seq;

    if (g.sats_used >= 4) {
      const float latRad = static_cast<float>(g.lat) * 1e-7f * DEG_TO_RAD;
      const float lonRad = static_cast<float>(g.lon) * 1e-7f * DEG_TO_RAD;

      if (!gpsRefSet) {
        refLatRad = latRad;
        refLonRad = lonRad;
        refAltM = g.alt_m;
        gpsRefSet = true;
      }

      const float dLat = latRad - refLatRad;
      const float dLon = lonRad - refLonRad;
      const float cosRefLat = cosf(refLatRad);

      const float northM = dLat * EARTH_RADIUS_M;
      const float eastM = dLon * EARTH_RADIUS_M * cosRefLat;
      const float downM = -(g.alt_m - refAltM);

      const float headingRad = g.heading_deg * DEG_TO_RAD;
      const float vNorth = g.speed_mps * cosf(headingRad);
      const float vEast = g.speed_mps * sinf(headingRad);
      const float vDown = 0.0f;

      kf.updateGPSPosVel(
        eskf::Vec3(northM, eastM, downM),
        eskf::Vec3(vNorth, vEast, vDown),
        GPS_SIGMA_POS_M,
        GPS_SIGMA_VEL_MPS
      );
    }
  }

  const unsigned long now = millis();

  // Throttle prints (still only printing *new* samples)
  if (now - lastPrintTime < PRINT_INTERVAL_MS) return;
  lastPrintTime = now;

  // Print: t_us, qw, qx, qy, qz (ESKF quaternion)
  Serial.print(now_us); Serial.print(",");
  Serial.print(kf.q.w, 3); Serial.print(",");
  Serial.print(kf.q.x, 3); Serial.print(",");
  Serial.print(kf.q.y, 3); Serial.print(",");
  Serial.println(kf.q.z, 3); Serial.print("\n");

  // Position
  // Serial.print(kf.p.x, 2); Serial.print(",");
  // Serial.print(kf.p.y, 2); Serial.print(",");
  // Serial.print(kf.p.z, 2); Serial.print("\n");

}