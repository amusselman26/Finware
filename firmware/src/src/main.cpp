#include <Arduino.h>
#include <Wire.h>

#include "services/SensorsFacade.hpp"  // Sensor facade
#include "drivers/SDLogger.h"          // sdBegin, openNextLog, writeSnapshot, flushLog, closeLog
#include "drivers/LoRaRadio.h"         // wrapper around RH_RF95 from earlier
#include "app/StateMachine.h"         // FSM
#include "drivers/FinDriver.h"         // add this
#include "app/RollPDController.h"      // roll PD controller

using namespace finware;

// ------------------- Pins / Addresses (adjust as needed) -------------------
constexpr uint8_t IMU_CS     = 10;     // BNO085 CS (SPI) — must not conflict with SD CS
constexpr uint8_t IMU_INT    = 9;      // BNO085 INT
constexpr int8_t  IMU_RST    = 5;      // BNO085 RST
constexpr uint8_t BARO_ADDR  = 0x5D;   // LPS22 I2C
constexpr uint8_t GNSS_ADDR  = 0x42;   // u-blox I2C

// LoRa (RFM95) — avoid conflict with SD CS=10 by NOT using 10 for RST/CS
constexpr uint8_t LORA_CS    = 12;
constexpr uint8_t LORA_INT   = 6;
constexpr uint8_t LORA_RST   = 13;
constexpr float   LORA_FREQ  = 915.0f; // MHz

// ------------------- Globals -------------------
char txtFilename[20];
char binFilename[20];

SensorsFacade sensors(IMU_CS, IMU_INT, IMU_RST, BARO_ADDR, GNSS_ADDR);
LoRaRadio     LoRa(LORA_CS, LORA_INT, LORA_RST, LORA_FREQ);
StateMachine  fsm;
FinDriver     fins;  // commented out to disable fin commands

// PD controller globals
// Tune these values for your airframe
RollPDParams g_rollParams = {
  .kp           = 0.6f,
  .kd           = 0.3f,
  .phi_cmd_rad  = 0.0f,          // hold zero roll until launch detected
  .u_max_rad        = 10.0f * static_cast<float>(M_PI) / 180.0f,   // +/- 10 deg
  .u_rate_max_rads  = 400.0f * static_cast<float>(M_PI) / 180.0f  // 400 deg/s
};

RollPDState g_rollState{false, 0.0f};
bool g_rollControlEnabled = false;   // becomes true after ARM command

// Minimal binary packet (little-endian) for LLA telemetry
// Must match the ground receiver's TelemetryLLA layout.
struct __attribute__((packed)) TelemetryLLA {
  int32_t  lat_e7;   // degrees * 1e7
  int32_t  lon_e7;   // degrees * 1e7
  float    alt_m;    // meters
  uint32_t t_ms;     // sender timestamp
};

struct __attribute__((packed)) FlightRecord {
  SensorsSnapshot snap;    // existing snapshot (unchanged contents)
  float fin_cmd_rad;       // commanded fin angle (roll command) [rad]
};

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // off until we know we're healthy
  delay(5000); // 5 seconds
  digitalWrite(LED_BUILTIN, LOW); // on to indicate we're starting setup
  Serial.begin(115200);
  while (!Serial) {}   // ok to keep setup prints for bring-up
  Serial.println("Finware Flight Computer Starting...");\

  Wire.begin();
  Serial.println("I2C initialized.");

  // --- Sensors ---
  if (!sensors.begin()) {
    Serial.println("Sensors init failed!");
    while (true) delay(1000);
  }
  Serial.println("Sensors initialized.");

  
  // --- LoRa ---
  if (!LoRa.begin(/*txPower=*/23)) {
    Serial.println("LoRa init failed!");
    while (true) delay(1000);
  }
  Serial.println("LoRa ready.");

  // --- SD ---
  if (!sdBegin()) {
    Serial.println("SD init failed!");
    while (true) delay(1000);
  }
  if (!openNextLog(binFilename, txtFilename)) {
    Serial.println("Failed to open next log file!");
    while (true) delay(1000);               
  }
  Serial.print("Logging to "); Serial.println(txtFilename);

  // --- Fins (disabled) ---
  Serial.println("I'm finning it");
  fins.begin();
  fins.finTestSequence(fins);   // run 0, -10, +10 test
  Serial.println("Fin test sequence complete.");

}

void loop() {
  static SystemState lastState = fsm.getState();
  
  // Check for LoRa commands (TEXT packets with 1-byte type header)
  String cmd;
  if (LoRa.receiveMessage(cmd, 1)) {      // 0 ms = poll, no blocking
    cmd.trim();
    String cmd_upper = cmd;
    cmd_upper.toUpperCase();
    Serial.print("LoRa RX: ");
    Serial.println(cmd);

    if (cmd_upper == "ARM") {
      fsm.setArmCommand(true);
        // After all sensors are initialized, switch IMU to gyro-integrated RV
      if (!sensors.setIMUReport(SH2_GYRO_INTEGRATED_RV, 5000)) {
        Serial.println("Failed to set IMU report to GYRO_INTEGRATED_RV");
      } else {
        LoRa.sendText("Armed. IMU report set to GYRO_INTEGRATED_RV.");
      }
      // Reset roll controller on ARM; it will be enabled on launch
      g_rollParams.phi_cmd_rad = 0.0f;  // start from zero-roll command when armed
      roll_pd_reset(g_rollState, 0.0f);
    }

    else if (cmd_upper == "ABORT") {
      fsm.transitionTo(SystemState::ABORT, &sensors); 
      LoRa.sendText("Abort command received. Transitioning to ABORT state.");
      // Disable roll controller and (was) drive fins to neutral on abort
      g_rollControlEnabled = false;
      fins.commandNeutral();
    }

    else if (cmd_upper == "CALIB_BARO") {
      sensors.baro_.calibrateAtm();
      sensors.calibrateGNSSAltitude();
      sensors.baro_.tick(); // update immediately after calibration
      float alt = sensors.baro().altitude_m;
      String msg = "Barometer calibrated. Current altitude: " + String(alt, 2) + " m";
      LoRa.sendText(msg);
    }

    else if (cmd_upper == "TEST") {
      fins.finTestSequence(fins);
      LoRa.sendText("Fin test sequence executed (fins disabled).");
    }
    
    else {
      Serial.println("LoRa: Unknown command");
    }
  }

  // Radio messages for state transitions
  SystemState current = fsm.getState();
  if (current != lastState) {
    String message = "STATE CHANGED TO: " + fsm.stateName(current);
    LoRa.sendText(message);
    // When launch is detected, begin rolling to 1.8 rad
    if (current == SystemState::LAUNCH) {
      g_rollControlEnabled = true; // Enable roll control only on launch
      g_rollParams.phi_cmd_rad = 1.8f;
      Serial.println("Launch detected: roll command set to 1.8 rad.");
    }
    lastState = current;   // <-- update so it won't repeat

    // Disable roll control at apogee
    if (current == SystemState::APOGEE) {
      g_rollControlEnabled = false; // Disable roll control at apogee
      Serial.println("Apogee detected: roll control disabled.");
    }
  }


  // Update sensors
  sensors.tick();
  fsm.update(sensors);

  // Log a full snapshot to SD (binary)
  const SensorsSnapshot snap_now = sensors.snapshot();

  const uint32_t nowMs = millis();

  // Roll controller
  // Use IMU quaternion + gyro X as roll angle and roll rate.
  static uint64_t lastImuTUs = 0;
  float phi_meas_rad = 0.0f;
  float p_meas_rads  = 0.0f;
  float u_cmd_rad    = 0.0f;

  const IMU_Sample& imu = sensors.imu();
  if (imu.t_us != 0) {
    const float qw = imu.q[0];
    const float qx = imu.q[1];
    const float qy = imu.q[2];
    const float qz = imu.q[3];

    // Quaternion -> roll (rad), X-axis, aerospace convention
    const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
    const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
    phi_meas_rad = atan2f(sinr_cosp, cosr_cosp);

    p_meas_rads = imu.gx;  // gyro X [rad/s]

    float dt_s = 0.0f;
    if (lastImuTUs != 0 && imu.t_us > lastImuTUs) {
      dt_s = static_cast<float>(imu.t_us - lastImuTUs) * 1e-6f;
    }
    lastImuTUs = imu.t_us;

    // Controller always runs, but only applies when g_rollControlEnabled=true
    u_cmd_rad = roll_pd_update(g_rollParams,
                               g_rollState,
                               phi_meas_rad,
                               p_meas_rads,
                               dt_s,
                               g_rollControlEnabled);
  } else {
    // No IMU data yet: keep controller in a benign state
    u_cmd_rad = roll_pd_update(g_rollParams,
                               g_rollState,
                               0.0f,
                               0.0f,
                               0.0f,
                               false);
  }

  static float u_sent_rad = 0.0f; // for logging
  // Fin command at 50 hz
  static uint32_t lastFinUpdateMs = 0;
  if (nowMs - lastFinUpdateMs >= 20u) {   // 50 Hz
    float angles[NUM_FINS];
    const float u_deg = u_cmd_rad * (180.0f / static_cast<float>(M_PI));
    for (int i = 0; i < NUM_FINS; ++i) {
      angles[i] = u_deg;   // same roll command to all fins (canted fins yield roll)
    }
    fins.setAllFinAngles(angles);
    lastFinUpdateMs = nowMs;
    u_sent_rad = u_cmd_rad; // for logging
    // Serial.println("Fin angles (deg): " + String(u_deg, 2));
  }

  // Remove before flight
  // --- Loops per second over Serial ---
  static uint32_t loopCount = 0;
  static uint32_t lastLpsMs = 0;
  loopCount++;
  if (nowMs - lastLpsMs >= 1000u) {
    Serial.print("Loops per second: ");
    Serial.println(loopCount);
    loopCount = 0;
    lastLpsMs = nowMs;
    Serial.println(g_rollParams.phi_cmd_rad);
  }

  // Periodic SD flush (1 s)
  static uint32_t lastFlushMs = 0;
  if (nowMs - lastFlushMs >= 1000u) {
    flushLog();
    lastFlushMs = nowMs;
  }

  // Send lat, lon, alt over LoRa every 1 s
  static uint32_t lastTxMs = 0;
  if (nowMs - lastTxMs >= 1000u) {
    TelemetryLLA pkt;
    // Degrees * 1e7 to match receiver expectations
    pkt.lat_e7 = static_cast<int32_t>(snap_now.gnss.lat * 1e7);
    pkt.lon_e7 = static_cast<int32_t>(snap_now.gnss.lon * 1e7);
    pkt.alt_m  = snap_now.baro.altitude_m;
    pkt.t_ms   = nowMs;

    // Send as a typed LLA packet (1-byte header + TelemetryLLA payload)
    LoRa.sendTyped(LoRaPacketType::LLA,
                   reinterpret_cast<const uint8_t*>(&pkt),
                   static_cast<uint8_t>(sizeof(pkt)));
    lastTxMs = nowMs;
  }

  // Log a combined flight record (snapshot + fin command) to SD
  FlightRecord rec;
  rec.snap = snap_now;  // copy existing snapshot contents
  rec.fin_cmd_rad  = u_sent_rad;

  if (!writeRecordEx(&rec, sizeof(rec))) {
    LoRa.sendText("Failed to write extend ed log record.");
  }
}
