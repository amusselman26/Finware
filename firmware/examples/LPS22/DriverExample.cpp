// Example must be copied into main.cpp of src before runtime.
// Storing example here prevents multiple definitions of setup and loop within platformio.
#include <Arduino.h>
#include "drivers/Baro_LPS22.hpp"

using namespace finware;
Baro_LPS22 baro;

void setup() {
  Serial.begin(115200);
  while (!Serial) { /* wait for USB CDC */ }

  Serial.println("\n[LPS22] bring-up...");
  Wire.begin();

  // Start with the more accurate rotation vector at ~200 Hz (5000 us interval)
  if (!baro.begin(LPS22_RATE_50_HZ)) {
    Serial.println("ERROR: LPS22 init failed");
    // You could continue without IM:
    while (true) { delay(1000); }
  }
  Serial.println("LPS22 ready.");
  baro.calibrateAtm();

}


void loop() {
    baro.tick();
    const auto &s = baro.latest();
    if (s.altitude_m < -100) {
        baro.calibrateAtm();
    }
    if (baro.ok()) {
        Serial.print("seq=");
        Serial.print(s.seq);
        Serial.print(" | P=");
        Serial.print(s.pressure_hPa, 2);
        Serial.print(" hPa T=");
        Serial.print(s.temperature_C, 2);
        Serial.print(" C Alt=");
        Serial.print(s.altitude_m, 2);
        Serial.println(" m");
        
    } else {
        Serial.println("LPS22 not healthy");
    }
}
