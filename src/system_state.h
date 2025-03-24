#pragma once

#include <Arduino.h>

class SystemState {
public:
    // Singleton instance
    static SystemState& getInstance() {
        static SystemState instance;
        return instance;
    }

    // System states
    enum class State { INITIALIZING, READY, RUNNING, ERROR, EMERGENCY_STOP };

    // Error codes
    enum class ErrorCode {
        NONE,
        MOTOR_INIT_FAILED,
        SPI_ERROR,
        THERMAL_WARNING,
        THERMAL_ERROR,
        DRIVER_ERROR,
        COMMUNICATION_ERROR
    };

    // State management
    void  setState(State newState);
    State getState() const {
        return currentState;
    }

    // Error handling
    void      setError(ErrorCode error);
    ErrorCode getError() const {
        return currentError;
    }
    const char* getErrorMessage() const;
    void        clearError();

    // Status checks
    bool isReady() const {
        return currentState == State::READY;
    }
    bool hasError() const {
        return currentState == State::ERROR;
    }
    bool isEmergencyStopped() const {
        return currentState == State::EMERGENCY_STOP;
    }

    // Temperature management
    void setTemperature(float temp) {
        currentTemperature = temp;
    }
    float getTemperature() const {
        return currentTemperature;
    }
    bool isOverheated() const;

private:
    SystemState()
        : currentState(State::INITIALIZING),
          currentError(ErrorCode::NONE),
          currentTemperature(0.0f) {}

    // Prevent copying
    SystemState(const SystemState&)            = delete;
    SystemState& operator=(const SystemState&) = delete;

    State     currentState;
    ErrorCode currentError;
    float     currentTemperature;
};