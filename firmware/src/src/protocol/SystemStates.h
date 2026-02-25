# pragma once

enum class SystemState {
    BOOT,
    IDLE,
    ARMED,
    LAUNCH,
    ASCENT,
    COAST,
    APOGEE,
    DESCENT,
    LANDED,
    SAFE,
    ABORT
};