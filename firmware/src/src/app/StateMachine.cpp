#include "StateMachine.h"
#include "drivers/logger.h" // Provides finware::Logger wrapper
#include "services/SensorsFacade.hpp"


using namespace finware;
// -----------------------------------------------------------
// Initialize the FSM
// -----------------------------------------------------------
void StateMachine::init() {
    currentState = SystemState::BOOT;
    Logger::logText("STATE", "BOOT");
    initialized = false;
}

// -----------------------------------------------------------
// Update function: called every loop
// -----------------------------------------------------------
void StateMachine::update(const SensorsFacade& sensors) {
    // Handle sensor-based conditions for transitions
    const auto& imu  = sensors.imu();
    const auto& baro = sensors.baro();
    const auto& gnss = sensors.gnss();

    // Example conditions
    switch (currentState) {

        case SystemState::BOOT:
            if (!initialized) {
                Logger::logText("INFO", "System initialization complete");
                transitionTo(SystemState::IDLE);
                initialized = true;
            }
            break;

        case SystemState::IDLE:
            // Wait for arm command (e.g., from ground station)
            if (armedCommanded) {
                transitionTo(SystemState::ARMED);
            }
            break;

        case SystemState::ARMED:
            // Detect launch: barometer altitude increase or IMU acceleration
            if (baro.altitude_m > liftoffAltitudeThreshold ||
                imu.az > launchAccelThreshold) {
                transitionTo(SystemState::LAUNCH);
            }
            break;

        case SystemState::LAUNCH:
            // Detect motor burnout: acceleration drops below threshold
            if (imu.az < burnoutAccelThreshold) {
                transitionTo(SystemState::ASCENT);
            }
            break;

        case SystemState::ASCENT:
            // Detect apogee: vertical velocity crosses zero
            if (baro.altitude_m < 1500) {  // fix this
                transitionTo(SystemState::APOGEE);
            }
            break;

        case SystemState::APOGEE:
            // Trigger drogue deployment here
            Logger::logText("DEPLOY", "Drogue chute fired");
            transitionTo(SystemState::DESCENT);
            break;

        case SystemState::DESCENT:
            // Detect landing: altitude stable for a few seconds
            if (baro.altitude_m < 3.0) {
                transitionTo(SystemState::LANDED);
            }
            break;

        case SystemState::LANDED:
            Logger::logText("INFO", "Touchdown confirmed");
            transitionTo(SystemState::SAFE);
            break;

        case SystemState::SAFE:
            // Do nothing; mission complete
            break;

        case SystemState::ABORT:
            Logger::logText("ERROR", "Abort state entered");
            break;
    }
}

// -----------------------------------------------------------
// Transition helper
// -----------------------------------------------------------
void StateMachine::transitionTo(SystemState newState) {
    if (newState == currentState) return;

    Logger::logText("STATE", "→ " + stateName(newState));
    currentState = newState;
    lastTransitionTime = millis();
}

// -----------------------------------------------------------
// Get readable state name
// -----------------------------------------------------------
String StateMachine::stateName(SystemState s) const {
    switch (s) {
        case SystemState::BOOT:    return "BOOT";
        case SystemState::IDLE:    return "IDLE";
        case SystemState::ARMED:   return "ARMED";
        case SystemState::LAUNCH:  return "LAUNCH";
        case SystemState::ASCENT:  return "ASCENT";
        case SystemState::APOGEE:  return "APOGEE";
        case SystemState::DESCENT: return "DESCENT";
        case SystemState::LANDED:  return "LANDED";
        case SystemState::SAFE:    return "SAFE";
        case SystemState::ABORT:   return "ABORT";
        default:                   return "UNKNOWN";
    }
}
