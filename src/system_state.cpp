#include "system_state.h"

void SystemState::setState(State newState) {
    // Log state transition
    if (currentState != newState) {
        Serial.print("State change: ");
        switch (newState) {
            case State::INITIALIZING:
                Serial.println("INITIALIZING");
                break;
            case State::READY:
                Serial.println("READY");
                break;
            case State::RUNNING:
                Serial.println("RUNNING");
                break;
            case State::ERROR:
                Serial.println("ERROR");
                break;
            case State::EMERGENCY_STOP:
                Serial.println("EMERGENCY STOP");
                break;
        }
    }
    currentState = newState;
}

void SystemState::setError(ErrorCode error) {
    currentError = error;
    if (error != ErrorCode::NONE) {
        setState(State::ERROR);
        Serial.print("Error: ");
        Serial.println(getErrorMessage());
    }
}

const char* SystemState::getErrorMessage() const {
    switch (currentError) {
        case ErrorCode::NONE:
            return "No error";
        case ErrorCode::MOTOR_INIT_FAILED:
            return "Motor initialization failed";
        case ErrorCode::SPI_ERROR:
            return "SPI communication error";
        case ErrorCode::THERMAL_WARNING:
            return "Temperature warning";
        case ErrorCode::THERMAL_ERROR:
            return "Temperature critical";
        case ErrorCode::DRIVER_ERROR:
            return "Driver error";
        case ErrorCode::COMMUNICATION_ERROR:
            return "Communication error";
        default:
            return "Unknown error";
    }
}

void SystemState::clearError() {
    currentError = ErrorCode::NONE;
    if (currentState == State::ERROR) {
        setState(State::READY);
    }
}

bool SystemState::isOverheated() const {
    const float TEMPERATURE_WARNING = 60.0f;  // Celsius
    return currentTemperature >= TEMPERATURE_WARNING;
}