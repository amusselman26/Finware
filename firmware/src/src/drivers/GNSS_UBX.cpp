#include "GNSS_UBX.hpp"

using namespace finware;

GNSS_UBX::GNSS_UBX(uint8_t i2c_addr)
: _gnss(), _update_rate_hz(0) {}

// Needs an external wire.begin() first
bool GNSS_UBX::begin(int update_rate_hz) {
    _update_rate_hz = update_rate_hz;

    if (!_gnss.begin()) {
        _healthy = false;
        return false;
    }
    _healthy = true;
    _gnss.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    _gnss.setNavigationFrequency(_update_rate_hz);
    _gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    return true;
}

void GNSS_UBX::tick() {
    if (!_healthy) return;

    if (_gnss.getPVT()) {
        _latest.t_us = micros();
        _latest.lat = _gnss.getLatitude();
        _latest.lon = _gnss.getLongitude();
        _latest.alt_m = _gnss.getAltitude() / 1000.0f - _zero_alt_m; // mm -> m
        _latest.speed_mps = _gnss.getGroundSpeed() / 1000.0f; // mm/s -> m/s
        _latest.heading_deg = _gnss.getHeading() / 1e5f; // deg * 1e-5 -> deg
        _latest.sats_used = _gnss.getSIV();
        _latest.seq = ++_seq;
    }
}
void GNSS_UBX::calibrateAltitude() {;
    tick();
    _zero_alt_m = _latest.alt_m;
}