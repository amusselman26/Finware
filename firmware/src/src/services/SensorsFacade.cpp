#include "SensorsFacade.hpp"

using namespace finware;
SensorsFacade::SensorsFacade(uint8_t imu_cs, uint8_t imu_int, int8_t imu_rst,
                             uint8_t baro_addr,
                             uint8_t gnss_addr)
    : imu_(imu_cs, imu_int, imu_rst), baro_(baro_addr), gnss_(gnss_addr) {}

bool SensorsFacade::begin() {
    bool ok = true;
    ok &= imu_.begin(SH2_ARVR_STABILIZED_RV, 5000); // 200 Hz quaternion
    ok &= baro_.begin(LPS22_RATE_10_HZ);            // 10 Hz baro
    ok &= gnss_.begin(40);                            // GNSS at 40 Hz update rate
    return ok;
}

void SensorsFacade::tick() {
    bool ok = true;
    imu_.tick();
    baro_.tick();
    gnss_.tick();
    last_.t_us = Clock::now();

    if (imu_.ok()) {
        last_.imu = imu_.latest();
    }

    if (baro_.ok()) {
        last_.baro = baro_.latest();
    }

    if (gnss_.ok()) {
        last_.gnss = gnss_.latest();
    }
}