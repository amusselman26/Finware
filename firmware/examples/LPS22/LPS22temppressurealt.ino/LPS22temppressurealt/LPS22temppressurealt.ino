#include <Wire.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>

// ---- Pick the I2C address based on your SDO wiring ----
#define LPS22_I2C_ADDR 0x5D   // use 0x5D if SDO tied to 3V


Adafruit_LPS22 lps;

// Global variable for atmospheric pressure calibration
float P0;

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

  // Try explicit address first
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
  // Data Rates for LPS22: 1, 10, 25, 50, 75 Hz
  lps.setDataRate(LPS22_RATE_10_HZ);

  sensors_event_t pressure, temp;
  unsigned long start_time =  millis();
  unsigned long loop_duration = 5000; // loop for 5 seconds
  float calib_pressure, sum_pressure = 0;
  unsigned int num_of_loops = 0;

  Serial.println("Calibrating...");
  
  // Average current temperature and set as atmospheric
  if (lps.getEvent(&pressure, &temp)) {
    for (int i = 0; (millis() - start_time) < loop_duration; i++) {
      calib_pressure = pressure.pressure;
      sum_pressure += calib_pressure;
      num_of_loops += 1;
      delay(50);
    }
  }
  P0 = sum_pressure / num_of_loops;
  Serial.print("Atmospheric Pressure: ");
  Serial.println(P0);

}

void loop() {
  sensors_event_t temp, pressure;
  float p = pressure.pressure;
  float altitude, exponent;
  if (lps.getEvent(&pressure, &temp)) {
    // Formula for converting pressure in hPa to altitude in meters
    exponent = pow (p / P0, 0.1902);
    altitude = 44330 * (1 - exponent);
    Serial.print("Temperature: ");
    Serial.print(temp.temperature, 2);
    Serial.print(" C   |   Pressure: ");
    Serial.print(pressure.pressure, 2);
    Serial.print(" hPa  |  Altitude: ");
    Serial.println(altitude, 2);
  } else {
    Serial.println("Read failed");
  }
  delay(100);
}

