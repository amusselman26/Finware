#include <Arduino.h>
#include "services/SensorsFacade.hpp"   // adjust path as needed

using namespace finware;

// -------------------
// Configure pins / addrs
// -------------------
constexpr uint8_t IMU_CS   = 11;   // example chip select pin for BNO085 (SPI mode)
constexpr uint8_t IMU_INT  = 9;    // example interrupt pin
constexpr int8_t  IMU_RST  = 5;    // example reset pin
constexpr uint8_t BARO_ADDR = 0x5C; // LPS22 I²C address
constexpr uint8_t GNSS_ADDR = 0x42; // u-blox I²C address

// -------------------
// Create facade
// -------------------
SensorsFacade sensors(IMU_CS, IMU_INT, IMU_RST, BARO_ADDR, GNSS_ADDR);

void setup() {
  Serial.begin(115200);
  while (!Serial) { } // wait for USB if needed

  delay(5000); // wait for things to stabilize
  Serial.println("Initializing sensors...");

  if (!sensors.begin()) {
    Serial.println("Sensors init failed!");
    delay(2000); // wait 2 sec and try again
  }
  Serial.println("Sensors initialized.");
}

void loop() {
  // Tick all sensors — should be called frequently
  sensors.tick();

  // Periodically dump snapshot
  static uint32_t t_next_print = 0;
  if (millis() >= t_next_print) {
    SensorsSnapshot snap = sensors.snapshot();

    Serial.print("time: "); Serial.print(snap.t_us);
    Serial.print(" | IMU seq: "); Serial.print(snap.imu.q[1]);
    Serial.print("Pressure: "); Serial.println(snap.baro.pressure_hPa);
    Serial.print(" | GNSS lat: "); Serial.print(snap.gnss.lat, 6);

    t_next_print = millis() + 200; // every 200 ms
  }
}