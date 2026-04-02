#pragma once
#include "drivers/Baro_LPS22.hpp"
#include "drivers/IMU_BNO085.hpp"
#include "drivers/GNSS_UBX.hpp"
#include "protocol/SystemStates.h"
#include "drivers/BatteryMonitor.hpp"

namespace finware {

    // Snapshot of all sensors data
    struct SensorsSnapshot {
        uint64_t t_us;  // Timestamp of the snapshot in microseconds
        SystemState state;
        float batteryVoltage; // in volts
        IMU_Sample imu;
        BARO_Sample baro;
        GNSS_Sample gnss;
    };
}

class SensorsFacade {
    public:
    SensorsFacade(uint8_t imu_cs, uint8_t imu_int, int8_t imu_rst,
                  uint8_t baro_addr,
                  uint8_t gnss_addr = 0x42);
    bool begin();
    void tick();
        // Return a copy of the latest snapshot
    
    // Update FSM state
    void setState(SystemState s) { last_.state = s; }

    // Reconfigure IMU report type and interval without restarting all sensors
    bool setIMUReport(sh2_SensorId_t report, uint32_t reportIntervalUs) {
        return imu_.setReport(report, reportIntervalUs);
    }

    // Re-zero GNSS altitude to the current GNSS altitude reading.
    void calibrateGNSSAltitude() { gnss_.calibrateAltitude(); }

    finware::SensorsSnapshot snapshot() const { return last_; }

    // Direct (read-only) access to last-good samples
    const finware::IMU_Sample& imu()  const { return last_.imu; }
    const finware::BARO_Sample& baro() const { return last_.baro; }
    const finware::GNSS_Sample& gnss()   const { return last_.gnss; }
    float getBatteryVoltage() const { return last_.batteryVoltage; }
    finware::Baro_LPS22 baro_;

    private:
    finware::IMU_BNO085 imu_;
    finware::GNSS_UBX gnss_;
    finware::BatteryMonitor batteryMonitor_{};
    uint32_t last_gnss_tick_us_ = 0;

    finware::SensorsSnapshot last_{};
};