#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include "sensors/SensorTypes.hpp"
#include "sensors/SensorsHealth.hpp"

namespace finware {  

struct GNSS_Sample {
    uint64_t t_us;       // timestamp from Clock::now()
    long lat;            // latitude in degrees * 10^7
    long lon;            // longitude in degrees * 10^7
    float alt_m;          // altitude in meters
    float speed_mps;      // speed over ground in m/s
    float heading_deg;    // heading of motion in degrees
    uint8_t sats_used;    // number of satellites used in solution
    uint32_t seq;         // monotonically increasing
};

class GNSS_UBX {
    public:
    // I2C bring-up
    explicit GNSS_UBX (uint8_t i2c_addr = 0x42);

    // Begin I2C comms at specified data rate (1, 5, 10, 20, 30, 40? hz)
    // SparkFun library supports up to 40 hz. 20 hz should be sufficient
    bool begin(int update_rate_hz);

    // Read at most one sensor event; update latest sample
    void tick();

    void calibrateAltitude();

    // Diagnostics
    bool ok() const {  return _healthy;  }
    const GNSS_Sample& latest() const {  return _latest;  }
    uint32_t sequence() const {  return _seq;  }
    int updateRate() const {  return _update_rate_hz;  }

    // Not yet implemented
    const GPS_Health& health() const {  return _health;  }

    float zeroAltitude() const {  return _zero_alt_m;  }

    private:
    // State
    GNSS_Sample _latest{0, 0, 0, 0, 0, 0, 0, 0};
    volatile uint32_t _seq = 0;
    bool _healthy = false;
    bool _auto_pvt_enabled = false;
    int _update_rate_hz;
    float _zero_alt_m = 0.0f;

    // Health counters
    GPS_Health _health{};

    // Driver
    SFE_UBLOX_GNSS _gnss;

};
} // namespace finware