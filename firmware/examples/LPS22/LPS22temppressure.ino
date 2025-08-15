// Feather STM32F405 + Adafruit LPS22 (I2C) - Pressure/Temp demo
// Wiring (I2C mode):
//   LPS22 VIN  -> Feather 3V
//   LPS22 GND  -> Feather GND
//   LPS22 SCK  -> Feather SCL
//   LPS22 SDI  -> Feather SDA
//   LPS22 CS   -> 3V  (forces I2C mode)
//   LPS22 SDO  -> GND (addr 0x5C)  OR  3V (addr 0x5D)
//   INT optional (not used here)

#include <Wire.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>

// ---- Pick the I2C address based on your SDO wiring ----
#define LPS22_I2C_ADDR 0x5C   // use 0x5D if SDO tied to 3V

Adafruit_LPS22 lps;

void setup() {
  // Bring up USB CDC serial (STM32F405 needs a moment to enumerate)
  Serial.begin(115200);
  delay(2000);
  while (!Serial) { delay(10); }

  Serial.println("Feather STM32F405 + LPS22 (I2C) test");

  // Start I2C on default SDA/SCL pins and keep it conservative first
  Wire.begin();
  Wire.setClock(100000);   // start at 100 kHz for robust bring-up
  delay(50);               // allow sensor time to boot

  // Try explicit address first (avoids flaky auto-detect timing)
  if (!lps.begin_I2C(LPS22_I2C_ADDR)) {
    // Fallback: try the alternate address in case SDO is the other way
    uint8_t alt = (LPS22_I2C_ADDR == 0x5C) ? 0x5D : 0x5C;
    Serial.print("Addr 0x"); Serial.print(LPS22_I2C_ADDR, HEX);
    Serial.println(" failed; trying alternate...");
    if (!lps.begin_I2C(alt)) {
      Serial.println("Failed to find LPS22 chip on I2C (0x5C/0x5D).");
      while (1) { delay(10); }
    }
  }
  Serial.println("LPS22 Found!");

  // Now that comms are stable, you can speed up the bus if you like
  Wire.setClock(400000);   // 400 kHz Fast-mode I2C

  // Set data rate
  lps.setDataRate(LPS22_RATE_10_HZ);
  Serial.print("Data rate set to: ");
  switch (lps.getDataRate()) {
    case LPS22_RATE_ONE_SHOT: Serial.println("One Shot / Power Down"); break;
    case LPS22_RATE_1_HZ:     Serial.println("1 Hz");                 break;
    case LPS22_RATE_10_HZ:    Serial.println("10 Hz");                break;
    case LPS22_RATE_25_HZ:    Serial.println("25 Hz");                break;
    case LPS22_RATE_50_HZ:    Serial.println("50 Hz");                break;
    case LPS22_RATE_75_HZ:    Serial.println("75 Hz");                break;
    default:                  Serial.println("?");                     break;
  }
}

void loop() {
  sensors_event_t temp, pressure;
  if (lps.getEvent(&pressure, &temp)) {
    Serial.print("Temperature: ");
    Serial.print(temp.temperature, 2);
    Serial.print(" C   |   Pressure: ");
    Serial.print(pressure.pressure, 2);
    Serial.println(" hPa");
  } else {
    Serial.println("Read failed");
  }
  delay(100);
}
