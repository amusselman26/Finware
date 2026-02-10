#include "drivers/SDLogger.h"

// Define global logger objects (single definition)
File binFile;
File txtFile;

// H.U.G.S. Banner
const char* hugs_banner =
"=====================================================\n"
"   H   H  U   U   GGGG   SSSS                      \n"
"   H   H  U   U  G      S                           \n"
"   HHHHH  U   U  G  GG   SSS                        \n"
"   H   H  U   U  G   G      S                       \n"
"   H   H   UUU    GGGG  SSSS                        \n"
"-----------------------------------------------------\n"
"   Humanitarian Utility Guided System (H.U.G.S.)     \n"
"   University of Miami 2025                          \n"
"=====================================================\n";

bool sdBegin() {
    Serial.println("Initializing SD...");
    while (!SD.begin()) {
        delay(10);
    }
    Serial.println("SD has init.");
    return true;
}

bool openNextLog(char *binOut, char *txtOut) {
    for (int i = 1; i <= MAX_LOGS; i++) {
        sprintf(binOut, "LOG%04d.BIN", i);
        sprintf(txtOut, "LOG%04d.TXT", i);
        if (!SD.exists(binOut) && !SD.exists(txtOut)) {
            binFile = SD.open(binOut, FILE_WRITE);
            txtFile = SD.open(txtOut, FILE_WRITE);

            if (binFile && txtFile) {
                Serial.print("[INFO] Opened logs: ");
                Serial.print(binOut);
                Serial.print(" + ");
                Serial.println(txtOut);

                txtFile.println(hugs_banner);
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
    size_t written = binFile.write(
        reinterpret_cast<const uint8_t*>(&snap),
        sizeof(SensorsSnapshot)
    );
    return written == sizeof(SensorsSnapshot);
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