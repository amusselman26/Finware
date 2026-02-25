#pragma once
#include <Arduino.h>
#include <STM32SD.h>
#include "services/SensorsFacade.hpp"

constexpr uint8_t SD_CS_PIN = 10; // Chip select pin
constexpr int MAX_LOGS = 9999; // Maximum number of log files
constexpr uint32_t FLUSH_INTERVAL_MS = 1000;

// Declarations only; definitions live in SDLogger.cpp
extern File binFile;
extern File txtFile;

using namespace finware;

bool sdBegin();
bool openNextLog(char *binOut, char *txtOut);
bool writeRecord(const SensorsSnapshot &snap);
void writeText(const char* tag, const String& msg);
void flushLog();
void closeLog();
bool writeRecordEx(const void* data, size_t len);
