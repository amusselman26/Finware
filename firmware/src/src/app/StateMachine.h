#pragma once
#include "protocol/SystemStates.h"
#include "services/SensorsFacade.hpp"

class StateMachine {
public:
    void init();
    // Accept a non-const reference so the state machine can update the SensorsFacade
    void update(SensorsFacade& sensors);
    // Allow passing the SensorsFacade to transitionTo so it can push the new state into the snapshot
    void transitionTo(SystemState newState, SensorsFacade* sensors = nullptr);
    SystemState getState() const { return currentState; }
    String stateName(SystemState s) const;

    void setArmCommand(bool val) { armedCommanded = val; }

private:
    SystemState currentState = SystemState::BOOT;
    bool initialized = false;
    bool armedCommanded = true; // Default to true for testing; set to false for radio
    uint32_t lastTransitionTime = 0;

    // Apogee detection helper function
    bool apogeeDetected(const SensorsFacade& sensors);
    float prevAlt = 0.0f;
    bool altInitialized = false;
    int apogeeDropCount = 0;
    uint32_t launchTimeMs = 0;

    // Thresholds (tune per vehicle)
    const float liftoffAltitudeThreshold = -5.0f;   // meters
    const float launchAccelThreshold = 20.0f;       // m/s^2
    const float burnoutAccelThreshold = 0.3f;      // g
};
