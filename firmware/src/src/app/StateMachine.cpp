#include "StateMachine.h"
#include "drivers/logger.h" // Provides finware::Logger wrapper
#include "services/SensorsFacade.hpp"

// Constants for apogee detection
constexpr float APOGEE_DROP_THRESHOLD = 0; // meters
constexpr int APOGEE_COUNT_REQUIRED = 2;
constexpr float MIN_APOGEE_ALT = 0.0f; // meters
constexpr uint32_t MIN_APOGEE_TIME_MS = 5000; // milliseconds after launch before apogee detection starts
constexpr uint32_t MAX_APOGEE_TIME_MS = 8000; // milliseconds after launch before apogee detection stops

using namespace finware;
// -----------------------------------------------------------
// Initialize the FSM
// -----------------------------------------------------------
void StateMachine::init() {
    currentState = SystemState::BOOT;
    Logger::logText("STATE", "BOOT");
    initialized = false;
}

bool StateMachine::apogeeDetected(const SensorsFacade& sensors) {
    // float currAlt = sensors.baro().altitude_m;

    // if (!altInitialized) {
    //     prevAlt = currAlt;
    //     altInitialized = true;
    //     return false;
    // }

    // Guards
    // if (millis() - launchTimeMs < MIN_APOGEE_TIME_MS) return false;
    if (millis() - launchTimeMs < MAX_APOGEE_TIME_MS) return false;
    else return true;
    // if (currAlt < MIN_APOGEE_ALT) return false;

    // if (currAlt < prevAlt - APOGEE_DROP_THRESHOLD) {
    //     apogeeDropCount++;
    // }
    // else {
    //     apogeeDropCount = 0;
    // }

    // prevAlt = currAlt;
    // return apogeeDropCount >= APOGEE_COUNT_REQUIRED;
}

// -----------------------------------------------------------
// Update function: called every loop
// -----------------------------------------------------------
void StateMachine::update(SensorsFacade& sensors) {
    // Ensure the SensorsFacade snapshot reflects the FSM state for this tick
    sensors.setState(currentState);

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
            if (imu.ax > launchAccelThreshold) {
                transitionTo(SystemState::LAUNCH);
                launchTimeMs = millis(); // Record launch time for apogee detection
            }
            break;

        case SystemState::LAUNCH:
            // Detect motor burnout: acceleration drops below threshold
            if (imu.ax < burnoutAccelThreshold) {
                transitionTo(SystemState::ASCENT);
            }
            break;

        case SystemState::ASCENT:
            // Detect apogee: vertical velocity crosses zero
            if (apogeeDetected(sensors)) {
                transitionTo(SystemState::APOGEE);
            }
            break;

        case SystemState::APOGEE:
            // Trigger drogue deployment here
            Logger::logText("DEPLOY", "Drogue chute fired");
            transitionTo(SystemState::DESCENT, &sensors);
            break;

        case SystemState::DESCENT:
            // Detect landing: altitude stable for a few seconds
            if (sqrt(pow(imu.ax, 2) + pow(imu.ay, 2) + pow(imu.az, 2)) < 0.2f &&
                baro.altitude_m < 5.0f) {
                transitionTo(SystemState::LANDED);
            }
            break;

        case SystemState::LANDED:
            Logger::logText("INFO", "Touchdown confirmed");
            transitionTo(SystemState::SAFE, &sensors);
            break;

        case SystemState::SAFE:
            // Do nothing; mission complete
            break;

        case SystemState::ABORT:
            Logger::logText("ERROR", "Abort state entered");
            break;
    }

    // Ensure snapshot reflects any state changes by end of update
    sensors.setState(currentState);
}

// -----------------------------------------------------------
// Transition helper
// -----------------------------------------------------------
void StateMachine::transitionTo(SystemState newState, SensorsFacade* sensors) {
    if (newState == currentState) return;

    Logger::logText("STATE", ("→ " + stateName(newState)).c_str());
    currentState = newState;
    lastTransitionTime = millis();

    // Push the new state into the SensorsFacade snapshot if provided
    if (sensors) {
        sensors->setState(newState);
    }
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
        default: return "UNKNOWN";
    }
}
