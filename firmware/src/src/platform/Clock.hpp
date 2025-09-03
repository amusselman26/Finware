#pragma once
#include <stdint.h>
#include <Arduino.h>

namespace finware{
// All system time stamps will be 64bit
using time_us_t = uint64_t;

class Clock {
    public:
        // casts micros uint32 to uint64
        static inline time_us_t now() {
            return static_cast<time_us_t>(micros());
        }
}
} //finware namespace