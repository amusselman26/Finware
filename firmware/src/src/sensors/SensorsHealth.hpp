#pragma once

#include <cstdint>
#include "platform/Clock.hpp"

namespace finware {
    
    struct StreamCounters {
        uint32_t produced = 0;
        uint32_t consumed = 0;
        uint32_t errors = 0;
        time_us_t last_t_us = 0;
    };

    struct BARO_Health {
        StreamCounters ctr;
    };

    struct IMU_Health {
        StreamCounters ctr;
    };

    struct GPS_Health {
        StreamCounters ctr;
        uint8_t last_sats_used = 0;
    };

    struct SensorsHealthSnapshot {
        time_us_t t_us;
        BARO_Health baro;
        IMU_Health imu;
        GPS_Health gps;
    };

    inline void markProduced(StreamCounters& c, time_us_t t_us) {  ++c.produced; c.last_t_us = t_us; }
    inline void markConsumed(StreamCounters& c) {  ++c.consumed;  }
    inline void markError(StreamCounters& c) {  ++c.errors;  }
}