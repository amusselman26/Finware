#include <Arduino.h>
#include <Wire.h>
#include "drivers/IMU_BNO085.hpp"
#include "drivers/Baro_LPS22.hpp"
#include "drivers/GNSS_UBX.hpp"
#include "drivers/SDLogger.h"
#include "app/eskf15.h"

using namespace finware;

// IMU instance (CS=10, INT=9, RST=5)
IMU_BNO085 imu(10, 9, 5);
Baro_LPS22 baro(0x5D);
GNSS_UBX gnss(0x42);
eskf::ESKF15 kf;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL_MS = 1000; // print at most every 100ms
const unsigned long LOG_INTERVAL_MS = 50; // 20 Hz SD logging
const unsigned long GPS_VEL_PRINT_INTERVAL_MS = 50; // 20 Hz GPS velocity print
const unsigned long GPS_FALLBACK_UPDATE_INTERVAL_MS = 33; // ~30 Hz
unsigned long loopRateWindowStartMs = 0;
uint32_t loopCount = 0;
uint32_t imuUpdateCount = 0;
unsigned long lastFallbackGpsUpdateMs = 0;
unsigned long lastGpsVelPrintMs = 0;

uint32_t prev_seq = 0;
uint32_t prev_baro_seq = 0;
uint32_t prev_gnss_seq = 0;
unsigned long prev_time_us = 0;

bool gpsRefSet = false;
float refLatRad = 0.0f;
float refLonRad = 0.0f;
float refAltM = 0.0f;

constexpr float EARTH_RADIUS_M = 6378137.0f;
constexpr float GPS_SIGMA_POS_M = 1.0f;
constexpr float GPS_SIGMA_VEL_MPS = 0.2f;
constexpr float GPS_HEADING_MIN_SPEED_MPS = 3.0f;
constexpr float BARO_SIGMA_ALT_M = 2.0f;

constexpr float GPS_FALLBACK_NORTH_M = 0.0f;
constexpr float GPS_FALLBACK_EAST_M = 0.0f;
constexpr float GPS_FALLBACK_DOWN_M = 0.0f;

char logBinName[16] = {0};
char logTxtName[16] = {0};
float last_r_baro = NAN;
float last_r_gps_pos_x = NAN;
float last_r_gps_pos_y = NAN;
float last_r_gps_pos_z = NAN;
float last_r_gps_vel_x = NAN;
float last_r_gps_vel_y = NAN;
float last_r_gps_vel_z = NAN;
uint32_t logRowCount = 0;
unsigned long lastLogTime = 0;
unsigned long lastSdFlushMs = 0;

static void quatToEulerDeg(const eskf::Quat& q, float& rollDeg, float& pitchDeg, float& yawDeg) {
  const float sinr_cosp = 2.0f * (q.w * q.x + q.y * q.z);
  const float cosr_cosp = 1.0f - 2.0f * (q.x * q.x + q.y * q.y);
  const float roll = atan2f(sinr_cosp, cosr_cosp);

  const float sinp = 2.0f * (q.w * q.y - q.z * q.x);
  float pitch;
  if (fabsf(sinp) >= 1.0f) {
    pitch = copysignf(HALF_PI, sinp);
  } else {
    pitch = asinf(sinp);
  }

  const float siny_cosp = 2.0f * (q.w * q.z + q.x * q.y);
  const float cosy_cosp = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
  const float yaw = atan2f(siny_cosp, cosy_cosp);

  rollDeg = roll * RAD_TO_DEG;
  pitchDeg = pitch * RAD_TO_DEG;
  yawDeg = yaw * RAD_TO_DEG;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Wire.begin();

  if (sdBegin() && openNextLog(logBinName, logTxtName) && txtFile) {
    txtFile.println("t_us,p_x,p_y,p_z,v_x,v_y,v_z,roll_deg,pitch_deg,yaw_deg,r_baro,r_gps_pos_x,r_gps_pos_y,r_gps_pos_z,r_gps_vel_x,r_gps_vel_y,r_gps_vel_z");
    txtFile.flush();
    // Serial.print("Logging TXT file: ");
    // Serial.println(logTxtName);
  } else {
    // Serial.println("SD logging disabled.");
  }

  // Serial.println("Starting ESKF predict...");

  // Request calibrated gyroscope (accel & gyro enabled at 5ms = 200 Hz)
  if (!imu.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1) delay(100);
  }

  // Wait for first rotation-vector quaternion sample.
  const uint32_t qseq0 = imu.quatSequence();
  const unsigned long qWaitStartMs = millis();
  while (imu.quatSequence() == qseq0 && (millis() - qWaitStartMs < 2000UL)) {
    imu.tick();
  }

  // Use the first available quaternion to initialize attitude.
  kf.q = eskf::Quat::enuBodyToNavToNedBodyToNav(
    eskf::Quat(
      imu.latest().q[0],
      imu.latest().q[1],
      imu.latest().q[2],
      imu.latest().q[3]
    )
  );

  // Disable rotation-vector streaming after initial attitude set.
  imu.disableRotationVectorReport();
  prev_seq = imu.latest().seq;
  prev_time_us = imu.latest().t_us;

  while (!gnss.begin(20)) {
    Serial.println("Failed to initialize GNSS!");
  }

  if (!baro.begin(LPS22_RATE_50_HZ)) {
    Serial.println("Failed to initialize BARO!");
  }

  baro.tick();
  if (baro.ok()) {
    baro.calibrateAtm();
  }
  prev_baro_seq = baro.latest().seq;

  gnss.tick();
  prev_gnss_seq = gnss.latest().seq;

  // Serial.println("IMU + ESKF initialized.");
  // Serial.println("t_us,qw,qx,qy,qz");

  loopRateWindowStartMs = millis();
}

void loop() {
  kf.heading_observable = false;
  loopCount++;
  const unsigned long nowMs = millis();
  if (nowMs - loopRateWindowStartMs >= 1000) {
    Serial.print("loops_per_sec,");
    Serial.println(loopCount);
    // Serial.print("imu_updates_per_sec,");
    // Serial.println(imuUpdateCount);
    loopCount = 0;
    imuUpdateCount = 0;
    loopRateWindowStartMs = nowMs;
  }

  imu.tick();

  const IMU_Sample& s = imu.latest();

  // Only act when a NEW sample arrives
  if (s.seq == prev_seq) return;
  prev_seq = s.seq;
  imuUpdateCount++;

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
  baro.tick();

  const BARO_Sample& b = baro.latest();
  if (baro.ok() && (b.seq != prev_baro_seq)) {
    prev_baro_seq = b.seq;
    if (!isnan(b.altitude_m)) {
      const float baroPredAltUp = -kf.p.z;
      last_r_baro = b.altitude_m - baroPredAltUp;
      kf.updateBaroAlt(b.altitude_m, BARO_SIGMA_ALT_M);
    }
  }

  const GNSS_Sample& g = gnss.latest();
  const bool gpsAvailable = gnss.ok();
  float gpsVx = NAN;
  float gpsVy = NAN;
  float gpsVz = NAN;
  if (gpsAvailable) {
    gpsVx = g.v_north_mps;
    gpsVy = g.v_east_mps;
    gpsVz = g.v_down_mps;
  }

  // Heading is observable only with a trusted heading cue (GPS course while moving).
  kf.heading_observable = gpsAvailable && (g.speed_mps > GPS_HEADING_MIN_SPEED_MPS);

  if (g.seq != prev_gnss_seq) {
    prev_gnss_seq = g.seq;

    if (gpsAvailable) {
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

      Serial.print(northM, 6); Serial.print(",");
      Serial.print(eastM, 6); Serial.print(",");
      Serial.println(downM, 6); Serial.print(",");

      const float vNorth = g.v_north_mps;
      const float vEast = g.v_east_mps;
      const float vDown = g.v_down_mps;

      const float rPosX = northM - kf.p.x;
      const float rPosY = eastM - kf.p.y;
      const float rPosZ = downM - kf.p.z;
      const float rVelX = vNorth - kf.v.x;
      const float rVelY = vEast - kf.v.y;
      const float rVelZ = vDown - kf.v.z;
      last_r_gps_pos_x = rPosX;
      last_r_gps_pos_y = rPosY;
      last_r_gps_pos_z = rPosZ;
      last_r_gps_vel_x = rVelX;
      last_r_gps_vel_y = rVelY;
      last_r_gps_vel_z = rVelZ;

      kf.updateGPSPosVel(
        eskf::Vec3(northM, eastM, downM),
        eskf::Vec3(vNorth, vEast, vDown),
        GPS_SIGMA_POS_M,
        GPS_SIGMA_VEL_MPS
      );
    }
  }

  if (!gpsAvailable && (nowMs - lastFallbackGpsUpdateMs >= GPS_FALLBACK_UPDATE_INTERVAL_MS)) {
    lastFallbackGpsUpdateMs = nowMs;
    const float rPosX = GPS_FALLBACK_NORTH_M - kf.p.x;
    const float rPosY = GPS_FALLBACK_EAST_M - kf.p.y;
    const float rPosZ = GPS_FALLBACK_DOWN_M - kf.p.z;
    const float rVelX = -kf.v.x;
    const float rVelY = -kf.v.y;
    const float rVelZ = -kf.v.z;
    last_r_gps_pos_x = rPosX;
    last_r_gps_pos_y = rPosY;
    last_r_gps_pos_z = rPosZ;
    last_r_gps_vel_x = rVelX;
    last_r_gps_vel_y = rVelY;
    last_r_gps_vel_z = rVelZ;
    kf.updateGPSPosVel(
      eskf::Vec3(GPS_FALLBACK_NORTH_M, GPS_FALLBACK_EAST_M, GPS_FALLBACK_DOWN_M),
      eskf::Vec3(0.0f, 0.0f, 0.0f),
      GPS_SIGMA_POS_M,
      GPS_SIGMA_VEL_MPS
    );
  }

  if (nowMs - lastGpsVelPrintMs >= GPS_VEL_PRINT_INTERVAL_MS) {
    lastGpsVelPrintMs = nowMs;
  }

  const unsigned long now = millis();

  if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
    lastPrintTime = now;

    const eskf::Vec3 f_b = eskf::Vec3(s.ax, s.ay, s.az) - kf.ba;
    const eskf::Vec3 a_n = kf.q.rotate(f_b);
    // Serial.print(a_n.x, 6); Serial.print(",");
    // Serial.print(a_n.y, 6); Serial.print(",");
    // Serial.print(a_n.z, 6); Serial.print(",");

    // Print: t_us, qw, qx, qy, qz (ESKF quaternion)
    // Serial.print(now_us); Serial.print(",");
    // Serial.print(kf.q.w, 3); Serial.print(",");
    // Serial.print(kf.q.x, 3); Serial.print(",");
    // Serial.print(kf.q.y, 3); Serial.print(",");
    // Serial.println(kf.q.z, 3); Serial.print("\n");

    // Position
    // Serial.print(kf.p.x, 2); Serial.print(",");
    // Serial.print(kf.p.y, 2); Serial.print(",");
    // Serial.print(kf.p.z, 2); Serial.print("\n");
  }

  if ((now - lastLogTime >= LOG_INTERVAL_MS) && txtFile) {
    lastLogTime = now;

    float rollDeg = 0.0f;
    float pitchDeg = 0.0f;
    float yawDeg = 0.0f;
    quatToEulerDeg(kf.q, rollDeg, pitchDeg, yawDeg);

    // txtFile.print(now_us); txtFile.print(",");
    // txtFile.print(kf.p.x, 6); txtFile.print(",");
    // txtFile.print(kf.p.y, 6); txtFile.print(",");
    // txtFile.print(kf.p.z, 6); txtFile.print(",");
    // txtFile.print(kf.v.x, 6); txtFile.print(",");
    // txtFile.print(kf.v.y, 6); txtFile.print(",");
    // txtFile.print(kf.v.z, 6); txtFile.print(",");
    // txtFile.print(rollDeg, 6); txtFile.print(",");
    // txtFile.print(pitchDeg, 6); txtFile.print(",");
    // txtFile.print(yawDeg, 6); txtFile.print(",");
    // txtFile.print(last_r_baro, 6); txtFile.print(",");
    // txtFile.print(last_r_gps_pos_x, 6); txtFile.print(",");
    // txtFile.print(last_r_gps_pos_y, 6); txtFile.print(",");
    // txtFile.print(last_r_gps_pos_z, 6); txtFile.print(",");
    // txtFile.print(last_r_gps_vel_x, 6); txtFile.print(",");
    // txtFile.print(last_r_gps_vel_y, 6); txtFile.print(",");
    // txtFile.println(last_r_gps_vel_z, 6);

    // Print whats below this
    // Serial.print(now_us); Serial.print(",");
    // Serial.print(kf.p.x, 6); Serial.print(",");
    // Serial.print(kf.p.y, 6); Serial.print(",");
    // Serial.print(kf.p.z, 6); Serial.print(",");
    // Serial.print(kf.v.x, 6); Serial.print(",");
    // Serial.print(kf.v.y, 6); Serial.print(",");
    // Serial.print(kf.v.z, 6); Serial.print(",");
    // Serial.print(rollDeg, 6); Serial.print(",");
    // Serial.print(pitchDeg, 6); Serial.print(",");
    // Serial.print(yawDeg, 6); Serial.print(",");
    // Serial.print(last_r_baro, 6); Serial.print(",");
    // Serial.print(last_r_gps_pos_x, 6); Serial.print(",");
    // Serial.print(last_r_gps_pos_y, 6); Serial.print(",");
    // Serial.print(last_r_gps_pos_z, 6); Serial.print(",");
    // Serial.print(last_r_gps_vel_x, 6); Serial.print(",");
    // Serial.print(last_r_gps_vel_y, 6); Serial.print(",");
    // Serial.print(last_r_gps_vel_z, 6); Serial.print(",");
    // Serial.println(gpsVx, 6);
    logRowCount++;
  }

  if (now - lastSdFlushMs >= FLUSH_INTERVAL_MS) {
    flushLog();
    lastSdFlushMs = now;
  }

}