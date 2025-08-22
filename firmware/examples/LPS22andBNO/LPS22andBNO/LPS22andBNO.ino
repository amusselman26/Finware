#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>

// ---- Pick the I2C address based on your SDO wiring ----
#define LPS22_I2C_ADDR 0x5C   // use 0x5D if SDO tied to 3V

#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_LPS22 lps;
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportInterValUs = 5000;

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void bnoSetup(void) {
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  setReports(reportType, reportInterValUs);
  delay(100);

  Serial.println("Reading events");
  delay(100);
}

void lps22Setup(void) {
  while (!Serial) { delay(10); }

  Serial.println("Feather STM32F405 + LPS22 (I2C) test");
  Wire.begin();
  Wire.setClock(100000);
  delay(50);

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

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void setup(void) {

  Serial.begin(115200);
  delay(2000);
  bnoSetup();
  lps22Setup();
}

void loop(void) {
  
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportInterValUs);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
  }

  static long last = 0;
  long now = micros();
  Serial.print(now - last);             Serial.print("\t");
  last = now;
  Serial.print(sensorValue.status);     Serial.print("\t");  // This is accuracy in the range of 0 to 3
  Serial.print(ypr.yaw);                Serial.print("\t");
  Serial.print(ypr.pitch);              Serial.print("\t");
  Serial.println(ypr.roll);

  sensors_event_t temp, pressure;
  float P0 = 1013.25;
  float p = pressure.pressure;
  float altitude, exponent;
  if (lps.getEvent(&pressure, &temp)) {
    exponent = pow (p / P0, 0.1902);
    altitude = 44330 * (1 - exponent);
    Serial.print("Temperature: ");
    Serial.print(temp.temperature, 2);
    Serial.print(" C   |   Pressure: ");
    Serial.print(pressure.pressure, 2);
    Serial.print(" hPa  |  Altitude: ");
    Serial.println(altitude, DEC);
  } else {
    Serial.println("Read failed");
  }
  delay(100);

}