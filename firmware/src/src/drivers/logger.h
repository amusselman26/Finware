#pragma once
#include "drivers/SDLogger.h"
#include "services/SensorsFacade.hpp"

namespace finware {
class Logger {
public:
    static bool begin() {
        return sdBegin();
    }

    static bool open() {
        return openNextLog(binName_, txtName_);
    }

    static void writeSnapshot(const finware::SensorsSnapshot &snap) {
        writeRecord(snap);
    }

    static void logText(const char* tag, const String &msg) {
        writeText(tag, msg);
    }

    static void flush() {
        flushLog();
    }

    static void close() {
        closeLog();
    }

private:
    static inline char binName_[16];
    static inline char txtName_[16];
};
} // namespace finware