#include <SPI.h>
#include "SdFat.h"
#include "services/SensorsFacade.hpp"

// -------------- CONFIG ----------------
constexpr uint8_t SD_CS_PIN = 10;
constexpr int MAX_LOGS = 9999;
SdFat sd;
File32 logFile;
// -----------------------------------------

using namespace finware;

char logFilename[20];

// ---- SD INIT ----
bool sdBegin() {
  if (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(18)))) {
    Serial.println("SD init failed!");
    sd.initErrorHalt();   // optional: print detailed error codes
    return false;
  }
  Serial.println("SD init OK.");
  return true;
}

// ---- AUTO FILE ROLLING ----
bool openNextLog(char *filenameOut) {
  for (int i = 1; i <= MAX_LOGS; i++) {
    sprintf(filenameOut, "LOG%04d.BIN", i);
    if (!sd.exists(filenameOut)) {
      if (logFile.open(filenameOut, O_WRITE | O_CREAT)) {
        Serial.print("Opened log file: ");
        Serial.println(filenameOut);
        return true;
      } else {
        Serial.println("File open failed!");
        return false;
      }
    }
  }
  Serial.println("No free log filenames!");
  return false;
}

// ---- SNAPSHOT WRITE ----
bool writeSnapshot(const SensorsSnapshot &snap) {
  int written = logFile.write(&snap, sizeof(snap));
  return written == sizeof(snap);
}

void flushLog() { logFile.flush(); }
void closeLog() { logFile.close(); }

// ---- DEMO ----
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  if (!sdBegin()) return;
  if (!openNextLog(logFilename)) return;

  // Example: write 10 dummy snapshots
  for (int i = 0; i < 10; i++) {
    SensorsSnapshot snap;
    snap.t_us = millis();
    snap.imu.q[1] = 0.01f * i;
    snap.imu.q[2] = 0.02f * i;
    snap.gnss.lat = 9.81f;
    snap.gnss.lon = -80.2789f;
    snap.gnss.alt_m = 5.0f + i;
    snap.baro.pressure_hPa = 0.1f * i;
    snap.baro.altitude_m = 1013.25f + i;

    if (writeSnapshot(snap)) {
      Serial.print("Wrote snapshot ");
      Serial.print(i);
      Serial.print(" to ");
      Serial.println(logFilename);
    } else {
      Serial.println("SD write error!");
    }

    delay(100); // simulate sample rate
  }

  flushLog();
  closeLog();
  Serial.println("Logging complete.");
}

void loop() {
  // In your real flight code: call writeSnapshot() once per cycle
}