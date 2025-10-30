#include "drivers/SDLogger.h"

// Define global logger objects (single definition)
SdFat sd;
File32 binFile;
File32 txtFile;

bool sdBegin() {
    if (!sd.begin(SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(18)))) {
        Serial.println("SD init failed!");
        sd.initErrorHalt();
        return false;
    }
    Serial.println("SD has init.");
    return true;
}

bool openNextLog(char *binOut, char *txtOut) {
    for (int i = 1; i <= MAX_LOGS; i++) {
        sprintf(binOut, "LOG%04d.BIN", i);
        sprintf(txtOut, "LOG%04d.TXT", i);
        if (!sd.exists(binOut) && !sd.exists(txtOut)) {
            if (binFile.open(binOut, O_WRITE | O_CREAT) &&
                txtFile.open(txtOut, O_WRITE | O_CREAT)) {

                Serial.print("[INFO] Opened logs: ");
                Serial.print(binOut);
                Serial.print(" + ");
                Serial.println(txtOut);

                // Write header to text log
                txtFile.println("=== H.U.G.S. Flight Log Start ===");
                txtFile.println("Firmware: Finware v1.3.2");
                txtFile.println("Vehicle: HUGS-01");
                txtFile.println("---------------------------------");
                txtFile.flush();

                return true;
            } else {
                Serial.println("[ERROR] Failed to open log files.");
                return false;
            }
        }
    }
    Serial.println("[ERROR] Max log files reached.");
    return false;
}

bool writeRecord(const SensorsSnapshot &snap) {
    if (!binFile) return false;
    int written = binFile.write(&snap, sizeof(snap));
    static uint32_t lastFlush = 0;
    if (millis() - lastFlush > FLUSH_INTERVAL_MS) {
        binFile.flush();
        lastFlush = millis();
    }
    return written == sizeof(snap);
}

void writeText(const char* tag, const String& msg) {
    if (!txtFile) return;
    uint32_t t_ms = millis();
    txtFile.printf("[%8lu ms] [%s] %s\n", t_ms, tag, msg.c_str());
    Serial.printf("[%8lu ms] [%s] %s\n", t_ms, tag, msg.c_str());
}

void flushLog() {
    if (binFile) binFile.flush();
    if (txtFile) txtFile.flush();
}

void closeLog() {
    if (txtFile) {
        txtFile.println("=== Flight Complete ===");
        txtFile.flush();
        txtFile.close();
    }

    if (binFile) binFile.close();
}