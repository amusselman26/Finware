#include <Wire.h>
#include "drivers/GNSS_UBX.hpp"

finware::GNSS_UBX gnss;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.begin();

  if (gnss.begin(10)) {
    Serial.println("GNSS started OK");
  } else {
    Serial.println("GNSS failed to start");
  }

  // Calibrate to current GNSS altitude
  gnss.calibrateAltitude();
}

void loop() {
  gnss.tick();
  auto s = gnss.latest();
  if (s.alt_m < -30) {
    gnss.calibrateAltitude();
  }
  Serial.print("Alt: "); Serial.print(s.alt_m);
  Serial.print(" (zero = "); Serial.print(gnss.zeroAltitude()); Serial.println(")");
  Serial.print(" Lat: "); Serial.print(s.lat * 1e-7, 7);
  Serial.print(" Lon: "); Serial.println(s.lon * 1e-7, 7);
  delay(50);
}
