#pragma once

#include <Adafruit_PWMServoDriver.h>

constexpr int NUM_FINS = 4;

enum FinID {
    FIN_TOP = 0,
    FIN_RIGHT,
    FIN_BOTTOM,
    FIN_LEFT
};

struct FinCalibration {
    float center_us;     // Neutral pulse width
    float us_per_deg;    // Gain
    float min_us;        // Mechanical limits
    float max_us;
    bool  reversed;
};

class FinDriver {
public:
    FinDriver(uint8_t i2c_addr = 0x40);

    void begin();

    void setFinAngle(FinID fin, float angle_deg);
    void setAllFinAngles(const float angles_deg[NUM_FINS]);

    void commandNeutral();
    void failsafe();

    void finTestSequence(FinDriver& fins) {
        float angles[NUM_FINS];

        // All fins to 0°
        for (int i = 0; i < NUM_FINS; ++i) {
            angles[i] = 0.0f;
        }
        fins.setAllFinAngles(angles);
        delay(1000);  // optional pause

        // All fins to -10°
        for (int i = 0; i < NUM_FINS; ++i) {
            angles[i] = -10.0f;
        }
        fins.setAllFinAngles(angles);
        delay(1000);  // optional pause

        // All fins to +10°
        for (int i = 0; i < NUM_FINS; ++i) {
            angles[i] = 10.0f;
        }
        fins.setAllFinAngles(angles);
        delay(1000);  // optional pause
    }

private:
    Adafruit_PWMServoDriver pwm_;

    FinCalibration cal_[NUM_FINS];

    float angleToPulse(FinID fin, float angle_deg);
};
