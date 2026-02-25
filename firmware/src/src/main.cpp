#include <Arduino.h>
#include "drivers/IMU_BNO085.hpp"
#include "app/ESKF15.hpp"
#include "drivers/Baro_LPS22.hpp"
#include "drivers/GNSS_UBX.hpp"

using namespace finware;

// Create IMU instance (CS=10, INT=9, RST=5)
IMU_BNO085 imu(10, 9, 5);

// ESKF (15-state) instance
finware::ESKF15 eskf;

// Barometer and GNSS instances
finware::Baro_LPS22 baro;
finware::GNSS_UBX gnss;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 1000; // Print every 1 second

// Track previous values for change detection
struct SensorChange {
  float prevGameRV[4] = {1, 0, 0, 0};  // Previous game rotation vector
  float prevAccel[3] = {0, 0, 0};      // Previous accel
  float prevGyro[3] = {0, 0, 0};       // Previous gyro
  uint32_t changeCount = 0;
};

SensorChange gameRVChange, accelChange, gyroChange;

const float EPSILON = 0.0001f;  // Threshold for detecting change

// ESKF timing/state
uint64_t prev_t_us = 0;
uint32_t prev_seq = 0;
uint32_t eskf_att_updates = 0;
uint32_t prev_baro_seq = 0;
uint32_t prev_gnss_seq = 0;

// GNSS local reference (for simple lat/lon -> NED conversion)
bool have_gnss_ref = true; // Set to true to use hardcoded reference. Set to false to use first GNSS fix as reference.
double ref_lat_rad = 0.0;
double ref_lon_rad = 0.0;
double ref_alt_m = 0.0;
const double DEG2RAD = 0.017453292519943295;
const double R_earth = 6378137.0; // meters

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("Starting IMU Frequency Monitor...");
  
  // Initialize IMU with game rotation vector at 5ms interval
  if (!imu.begin(SH2_GAME_ROTATION_VECTOR, 5000)) {
    Serial.println("Failed to initialize IMU!");
    while (1) delay(100);
  }
  
  Serial.println("IMU initialized successfully");

  // Prime IMU once and initialize ESKF nominal state
  imu.tick();
  const IMU_Sample& s = imu.latest();
  finware::Vec3 p0{0.0f, 0.0f, 0.0f};
  finware::Vec3 v0{0.0f, 0.0f, 0.0f};
  finware::Quat q0{s.q[0], s.q[1], s.q[2], s.q[3]};
  finware::Vec3 bg0{0.0f, 0.0f, 0.0f};
  finware::Vec3 ba0{0.0f, 0.0f, 0.0f};
  eskf.reset(p0, v0, q0, bg0, ba0);
  prev_t_us = s.t_us;
  prev_seq = s.seq;

  // Initialize barometer and GNSS
  if (!baro.begin(LPS22_RATE_50_HZ)) {
    Serial.println("Barometer failed to start");
  } else {
    Serial.println("Barometer initialized");
  }

  if (!gnss.begin(10)) {
    Serial.println("GNSS failed to start");
  } else {
    Serial.println("GNSS initialized");
  }
}

void loop() {
  imu.tick();
  
  const IMU_Sample& sample = imu.latest();
  unsigned long now = millis();

  // Compute dt from IMU timestamp (microseconds)
  float dt = 0.0f;
  if (prev_t_us != 0 && sample.t_us > prev_t_us) {
    dt = float(sample.t_us - prev_t_us) * 1e-6f;
  }

  // Run predict step if we have a non-zero dt
  if (dt > 0.0f) {
    finware::Vec3 gyro{sample.gx, sample.gy, sample.gz};
    finware::Vec3 accel{sample.ax, sample.ay, sample.az};
    eskf.predict(dt, gyro, accel);
  }
  
  // Tick baro & gnss drivers
  baro.tick();
  gnss.tick();

  // Handle barometer update
  {
    const auto& b = baro.latest();
    if (b.seq != prev_baro_seq) {
      finware::BaroMeas bm;
      bm.valid = true;
      // convert altitude (meters up) -> NED down positive
      bm.z_down_m = -b.altitude_m;
      eskf.updateBaro(bm);
      prev_baro_seq = b.seq;
    }
  }

  // Handle GNSS update (simple local NED conversion)
  {
    const auto& g = gnss.latest();
    //if (g.seq != prev_gnss_seq) {
      if (!have_gnss_ref) {
        ref_lat_rad = (double)g.lat * 1e-7 * DEG2RAD;
        ref_lon_rad = (double)g.lon * 1e-7 * DEG2RAD;
        ref_alt_m = (double)g.alt_m;
        have_gnss_ref = true;
      }

      double lat_rad = (double)g.lat * 1e-7 * DEG2RAD;
      double lon_rad = (double)g.lon * 1e-7 * DEG2RAD;
      double dlat = lat_rad - ref_lat_rad;
      double dlon = lon_rad - ref_lon_rad;
      double north = dlat * R_earth;
      double east = dlon * R_earth * cos(ref_lat_rad);
      double down = ref_alt_m - (double)g.alt_m;

      finware::GPSMeas gm;
      gm.valid = true;
      gm.p_ned_m.x = (float)north;
      gm.p_ned_m.y = (float)east;
      gm.p_ned_m.z = (float)down;

      double hdg_rad = (double)g.heading_deg * DEG2RAD;
      double vn = (double)g.speed_mps * cos(hdg_rad);
      double ve = (double)g.speed_mps * sin(hdg_rad);
      gm.v_ned_mps.x = (float)vn;
      gm.v_ned_mps.y = (float)ve;
      gm.v_ned_mps.z = 0.0f;

      eskf.updateGPS(gm);
      prev_gnss_seq = g.seq;
    //}
  }
  

  // If we have a new IMU sample sequence, use quaternion as attitude measurement
  if (sample.seq != prev_seq) {
    finware::AttMeas am;
    am.valid = true;
    am.q_nb = finware::Quat{sample.q[0], sample.q[1], sample.q[2], sample.q[3]};
    if (eskf.updateAttitude(am)) {
      eskf_att_updates++;
    }
    prev_seq = sample.seq;
    prev_t_us = sample.t_us;
  }
  
  // Print every second
  if (now - lastPrintTime >= PRINT_INTERVAL) {
    lastPrintTime = now;
    
    // Print a simple ESKF state summary: position (NED) and attitude quaternion
    finware::Vec3 p = eskf.positionNED();
    finware::Quat q = eskf.attitude_q_nb();
    Serial.print(p.x, 3); Serial.print(","); Serial.print(p.y, 3); Serial.print(","); Serial.print(p.z, 3);
    Serial.print(" ");
    Serial.print(q.w, 3); Serial.print(","); Serial.print(q.x, 3); Serial.print(","); Serial.print(q.y, 3); Serial.print(","); Serial.println(q.z, 3);

  }
}