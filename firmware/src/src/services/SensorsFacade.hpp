#pragma once
#include "platform/Clock.hpp"
#include "drivers/Baro_LPS22.hpp"
#include "drivers/IMU_BNO085.hpp"
#include "drivers/GNSS_UBX.hpp"

namespace finware {

    // Snapshot of all sensors data
    struct SensorsSnapshot {
        time_us_t t_us;  // Timestamp of the snapshot in microseconds
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
    
    finware::SensorsSnapshot snapshot() const { return last_; }

    // Direct (read-only) access to last-good samples
    const finware::IMU_Sample& imu()  const { return last_.imu; }
    const finware::BARO_Sample& baro() const { return last_.baro; }
    const finware::GNSS_Sample& gnss()   const { return last_.gnss; }

    private:
    finware::IMU_BNO085 imu_;
    finware::Baro_LPS22 baro_;
    finware::GNSS_UBX gnss_;

    finware::SensorsSnapshot last_{};
};