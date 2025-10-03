#pragma once
#include <SPI.h>
#include "SdFat.h"
#include "services/SensorsFacade.hpp"

constexpr uint8_t SD_CS_PIN = 10; // Chip select pin
constexpr int MAX_LOGS = 9999; // Maximum number of log files
SdFat sd;
File32 logFile;

using namespace finware;

bool sdBegin() {
    if (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(18)))) {
        Serial.println("SD init failed!");
        sd.initErrorHalt();
        return false;
    }
    Serial.println("SD has init.");
    return true;
}

bool openNextLog(char *filenameOut) {
    for (int i = 1; i <= MAX_LOGS; i++) {
        sprintf(filenameOut, "LOG%04d.BIN", i);
        if (!sd.exists(filenameOut)) {
            if (logFile.open(filenameOut, O_WRITE | O_CREAT)) {
                Serial.print("Opened file: ");
                Serial.println(filenameOut);
                return true;
            } else {
                Serial.print("Error opening file: ");
                Serial.println(filenameOut);
                return false;
            }
        }
    }
    Serial.println("Max log files reached.");
    return false;
}

bool writeRecord(const SensorsSnapshot &snap) {
    int written = logFile.write(&snap, sizeof(snap));
    return written == sizeof(snap);
}

void flushLog() {
    logFile.flush();
}

void closeLog() {
    logFile.close();
}
