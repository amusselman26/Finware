#pragma once
#include <Arduino.h>

#define VBATPIN A6

namespace finware {

    class BatteryMonitor {
        public:

        BatteryMonitor(uint8_t pin=VBATPIN);

        void begin();
        float readVoltage() const;

        private:
        uint8_t _pin;
    };
}