#include "SensorsFacade.hpp"
#include "protocol/SystemStates.h" // ensure SystemState is available

using namespace finware;
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
    return ok;
}

void SensorsFacade::calibrateAltitudeReferences() {
    if (baro_.ok()) {
        baro_.tick();
        baro_.calibrateAtm();
        baro_.tick();
        last_.baro = baro_.latest();
    }

    if (gnss_.ok()) {
        gnss_.calibrateAltitude();
        last_.gnss = gnss_.latest();
    }

    last_.t_us = micros();
}

void SensorsFacade::tick() {
    bool ok = true;
    imu_.tick();
    baro_.tick();
    gnss_.tick();
    last_.t_us = micros();

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