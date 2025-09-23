#include "Baro_LPS22.hpp"

using namespace finware;

Baro_LPS22::Baro_LPS22(uint8_t i2c_addr)
: _i2c_addr(i2c_addr) {}

// Needs an external wire.begin() first 
bool Baro_LPS22::begin(lps22_rate_t data_rate) {
    _data_rate = data_rate;

    if (!_lps22.begin_I2C(_i2c_addr)) {
        _healthy = false;
        return false;
    }

    _healthy = true;
    _lps22.setDataRate(data_rate);
    return true;
}

void Baro_LPS22::tick() {
    if (!_healthy) return;

    sensors_event_t pressure, temp;
    if (_lps22.getEvent(&pressure, &temp)) {
        _latest.t_us = micros();
        _latest.pressure_hPa = pressure.pressure;
        _latest.temperature_C = temp.temperature;
        _latest.altitude_m = Baro_LPS22::pressureToAltitude(_latest.pressure_hPa, _sea_level_hPa);
        _latest.seq = ++_seq;
    }
}

void Baro_LPS22::calibrateAtm() {
    Baro_LPS22::tick();

    // Use latest pressures as sea-level reference
    if (_latest.pressure_hPa > 0) {
        _sea_level_hPa = _latest.pressure_hPa;
    }
}

float Baro_LPS22::pressureToAltitude(float pressure_hPa, float seaLevel_hPa) {
    // Barometric formula
    // https://en.wikipedia.org/wiki/Barometric_formula
    // Simplified for constant temperature (valid for small altitude changes)
    // h = (T0 / L) * [ (P / P0)^(-R*L/gM) - 1 ]
    // where:
    // h = altitude (m)
    // T0 = standard temperature at sea level = 288.15 K
    // L = temperature lapse rate = 0.0065 K/m
    // P = measured pressure (hPa)
    // P0 = sea-level standard atmospheric pressure (hPa)
    // R = universal gas constant = 8.31432 N·m / (mol·K)
    // g = standard gravity = 9.80665 m/s²
    // M = molar mass of Earth's air = 0.0289644 kg/mol

    const float T0 = 288.15f;      // K
    const float L = 0.0065f;       // K/m
    const float R = 8.31432f;      // N·m / (mol·K)
    const float g = 9.80665f;      // m/s²
    const float M = 0.0289644f;    // kg/mol;
    float outAltitude_m;

    if (pressure_hPa <= 0 || seaLevel_hPa <= 0) {
        outAltitude_m = NAN;
        return outAltitude_m;
    }

    float exponent = (-R * L) / (g * M);
    outAltitude_m = (T0 / L) * (pow((pressure_hPa / seaLevel_hPa), exponent) - 1.0f);
    return outAltitude_m;
}

