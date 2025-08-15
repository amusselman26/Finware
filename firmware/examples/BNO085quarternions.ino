#include <Adafruit_BNO08x.h>
#include <Wire.h>

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Wire.begin();
  Wire.setClock(400000);               // you can raise to 400 kHz once stable
  delay(50);                           // let sensor boot

  // Try explicit address if auto detect fails:
  if (!bno08x.begin_I2C(0x4A)) {       // try 0x4B if needed
    Serial.println("Failed to find BNO085!");
    while (1) delay(10);
  }
  Serial.println("BNO085 found!");

  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
}

void loop() {
  sh2_SensorValue_t v;
  if (bno08x.getSensorEvent(&v) && v.sensorId == SH2_ROTATION_VECTOR) {
    Serial.print("Q: ");
    Serial.print(v.un.rotationVector.real, 4); Serial.print(", ");
    Serial.print(v.un.rotationVector.i, 4);    Serial.print(", ");
    Serial.print(v.un.rotationVector.j, 4);    Serial.print(", ");
    Serial.println(v.un.rotationVector.k, 4);
  }
}
