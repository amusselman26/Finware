#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LPS2X.h>

#include "platform/Clock.hpp"
#include "sensors/SensorTypes.hpp"
#include "sensors/SensorsHealth.hpp"

namespace finware {

// Sample struct for barometer (POD, trivially copyable)
struct BARO_Sample {
    time_us_t t_us;        // timestamp from Clock::now()
    float pressure_hPa;    // pressure in hectopascals
    float temperature_C;   // temperature in Celsius
    float altitude_m;      // derived altitude (optional, can be NAN)
    uint32_t seq;          // monotonically increasing
};

class Baro_LPS22 {
public:
    // I2C bring-up
    explicit Baro_LPS22(uint8_t i2c_addr = 0x5D);

    // Begin I2C comms at specified data rate (1, 10, 25, 50, 75 Hz)
    bool begin(lps22_rate_t data_rate);

    // Read at most one sensor event; update latest sample
    void tick();

    // Capture ambient pressure as sea-level reference (optional)
    void calibrateAtm();

    // Diagnostics
    bool ok() const { return _healthy; }
    const BARO_Sample& latest() const { return _latest; }
    uint32_t sequence() const { return _seq; }
    lps22_rate_t dataRate() const { return _data_rate; }

    const BARO_Health& health() const { return _health; }

private:
    // State
    BARO_Sample _latest{0, 0, 0, 0, 0};
    volatile uint32_t _seq = 0;
    bool _healthy = false;
    float _sea_level_hPa = 1013.25f; // default standard atmosphere

    // pressure to altitude conversion
    static float pressureToAltitude(float pressure_hPa, float seaLevel_hPa);

    // Health counters
    BARO_Health _health{};

    // Driver
    Adafruit_LPS22 _lps22;
    const uint8_t _i2c_addr;
    lps22_rate_t _data_rate;
};

} // namespace finware
