#include "SensorsFacade.hpp"
#include "protocol/SystemStates.h" // ensure SystemState is available

using namespace finware;

namespace {
constexpr uint32_t kGnssTickIntervalUs = 40000; // 25 Hz
}

SensorsFacade::SensorsFacade(uint8_t imu_cs, uint8_t imu_int, int8_t imu_rst,
                             uint8_t baro_addr,
                             uint8_t gnss_addr)
    : imu_(imu_cs, imu_int, imu_rst), baro_(baro_addr), gnss_(gnss_addr) {
    // Initialize snapshot state
    last_.state = SystemState::BOOT;
}

bool SensorsFacade::begin() {
    bool ok = true;
    ok &= imu_.begin(); // 200 Hz quaternion
    if (!ok) {
        Serial.println("IMU failed to start.");

    }
    delay(200);
    Serial.println("IMU initialized.");
    ok &= baro_.begin(LPS22_RATE_50_HZ);            // 50 Hz baro
    if (!ok) {
        Serial.println("Barometer failed to start.");
    }
    Serial.println("Barometer initialized.");
    ok &= gnss_.begin(40);                            // GNSS at 40 Hz update rate
    if (!ok) {
        Serial.println("GNSS failed to start.");
    }
    Serial.println("GNSS initialized.");

    // Initialize battery monitor
    batteryMonitor_.begin();
    last_.batteryVoltage = batteryMonitor_.readVoltage();
    return ok;
}

void SensorsFacade::tick() {
    const uint32_t now_us = micros();

    imu_.tick();
    baro_.tick();
    if (static_cast<uint32_t>(now_us - last_gnss_tick_us_) >= kGnssTickIntervalUs) {
        gnss_.tick();
        last_gnss_tick_us_ = now_us;
    }

    last_.t_us = now_us;

    if (imu_.ok()) {
        last_.imu = imu_.latest();
    }

    if (baro_.ok()) {
        last_.baro = baro_.latest();
    }

    if (gnss_.ok()) {
        last_.gnss = gnss_.latest();
    }

    last_.batteryVoltage = batteryMonitor_.readVoltage();

    // last_.state is preserved and can be updated externally via setState(...)
}