#include "FinDriver.h"
#include <algorithm>

constexpr float SERVO_FREQ_HZ = 50.0f;

static inline float clamp(float x, float lo, float hi) {
    return std::max(lo, std::min(x, hi));
}

FinDriver::FinDriver(uint8_t i2c_addr)
    : pwm_(i2c_addr)
{
    // ---- CALIBRATION VALUES (example) ----
    cal_[FIN_TOP] = {
        .center_us  = 1470,
        .us_per_deg = 10.0f,
        .min_us     = 1200,
        .max_us     = 1800,
        .reversed   = false
    };

    cal_[FIN_RIGHT] = {
        .center_us  = 1525,
        .us_per_deg = 10.0f,
        .min_us     = 1200,
        .max_us     = 1800,
        .reversed   = false
    };

    cal_[FIN_BOTTOM] = {
        .center_us  = 1395,
        .us_per_deg = 10.0f,
        .min_us     = 1200,
        .max_us     = 1800,
        .reversed   = false
    };

    cal_[FIN_LEFT] = {
        .center_us  = 1560,
        .us_per_deg = 10.0f,
        .min_us     = 1200,
        .max_us     = 1800,
        .reversed   = false
    };
}

void FinDriver::begin() {
    pwm_.begin();
    pwm_.setPWMFreq(SERVO_FREQ_HZ);
    delay(10);

    commandNeutral();
}

float FinDriver::angleToPulse(FinID fin, float angle_deg) {
    const FinCalibration& c = cal_[fin];

    float sign = c.reversed ? -1.0f : 1.0f;
    float us = c.center_us + sign * angle_deg * c.us_per_deg;

    return clamp(us, c.min_us, c.max_us);
}

void FinDriver::setFinAngle(FinID fin, float angle_deg) {
    float us = angleToPulse(fin, angle_deg);
    pwm_.writeMicroseconds(static_cast<uint8_t>(fin), static_cast<uint16_t>(us));
}

void FinDriver::setAllFinAngles(const float angles_deg[NUM_FINS]) {
    for (int i = 0; i < NUM_FINS; ++i) {
        setFinAngle(static_cast<FinID>(i), angles_deg[i]);
    }
}

void FinDriver::commandNeutral() {
    for (int i = 0; i < NUM_FINS; ++i) {
        pwm_.writeMicroseconds(i, static_cast<uint16_t>(cal_[i].center_us));
    }
}

void FinDriver::failsafe() {
    commandNeutral();
}
