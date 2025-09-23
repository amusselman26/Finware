#include "drivers/SDCard.hpp"
#include "drivers/IMU_BNO085.hpp"

using namespace finware;

SDCard sd;

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  while (!sd.begin()) {
    Serial.println("SD mount failed");
    return;
  }
  if (!sd.open("/flight.bin")) {
    Serial.println("open failed");
    return;
  }
  sd.println("LOG START");
}

void loop() {
  // Example: write a POD struct directly
  IMU_Sample s{};
  s.t_us = micros();
  s.q[0] = 1; s.q[1] = 0; s.q[2] = 0; s.q[3] = 0;

  sd.write(&s, sizeof(s));  // blocking write

  delay(100);
}