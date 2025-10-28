#pragma once
#include "protocol/SystemStates.h"
#include "services/SensorsFacade.hpp"

class StateMachine {
public:
    void init();
    void update(const SensorsFacade& sensors);
    void transitionTo(SystemState newState);
    SystemState getState() const { return currentState; }
    String stateName(SystemState s) const;

    void setArmCommand(bool val) { armedCommanded = val; }

private:
    SystemState currentState = SystemState::BOOT;
    bool initialized = false;
    bool armedCommanded = false;
    uint32_t lastTransitionTime = 0;

    // Thresholds (tune per vehicle)
    const float liftoffAltitudeThreshold = 5.0f;   // meters
    const float launchAccelThreshold = 2.0f;       // g
    const float burnoutAccelThreshold = 0.3f;      // g
};
