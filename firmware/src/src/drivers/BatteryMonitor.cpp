#include "BatteryMonitor.hpp"

namespace finware {

    BatteryMonitor::BatteryMonitor(uint8_t pin) 
    : _pin(pin)
    {}

    void BatteryMonitor::begin() {
        pinMode(_pin, INPUT);
    }

    float BatteryMonitor::readVoltage() const {
        // From adafruit guide under Measuring Battery section:
        float raw = analogRead(_pin);
        raw *= 2;
        raw *= 3.3; // Multiply by 3.3V, reference voltage
        float voltage = raw / 1024.0; // convert to voltage
        return voltage;
    }

}