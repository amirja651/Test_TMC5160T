/**
 * @brief Main program entry point
 * @file main.cpp
 */
#include <Arduino.h>
#include "command_handler.h"
#include "motor_controller.h"
#include "system_state.h"

// Global instances
MotorController motor;
CommandHandler  commander(motor);

// Add this forward declaration at the top of the file
void handleFatalError(const char* errorMsg);

/**
 * @brief Arduino setup function - Initializes system components
 */
void setup() {
    Serial.begin(Config::Timing::SERIAL_BAUDRATE);
    delay(Config::Timing::INIT_DELAY_MS);

    auto& state = SystemState::getInstance();
    state.setState(SystemState::State::INITIALIZING);

    Serial.println("\n== Initializing System ==\n");

    // Initialize motor controller
    if (!motor.initialize()) {
        state.setError(SystemState::ErrorCode::MOTOR_INIT_FAILED);
        handleFatalError("Motor initialization failed");
    }

    // Verify SPI communication
    auto testResult = motor.testConnection();
    if (!testResult.success) {
        state.setError(SystemState::ErrorCode::SPI_ERROR);
        Serial.println("SPI Connection Test Failed:");
        Serial.println(testResult.errorMessage);
        handleFatalError("SPI connection failed");
    }

    // Log successful connection
    Serial.println("SPI Connection Test Passed");
    Serial.print("Driver Version: 0x");
    Serial.println(testResult.version, HEX);

    // Configure motor parameters
    if (!motor.configure()) {
        handleFatalError("Motor configuration failed");
    }

    // System ready
    Serial.println("\nCommands (single letter):");
    Serial.println("mm <steps> <r/l> - Move");
    Serial.println("ss <speed>       - Speed");
    Serial.println("sc <current>     - Current");
    Serial.println("st               - Status");
    Serial.println("sd               - Detailed Status");
    Serial.println("em               - Enable/Disable");
    Serial.println("dm               - Disable");
    Serial.println("tc               - Test Connection");
    Serial.println("sh               - Show Help");
    Serial.println("es               - Emergency Stop");
    Serial.println("rt               - Read motor temperature");

    state.setState(SystemState::State::READY);
}

/**
 * @brief Arduino main loop - Processes real-time operations
 */
void loop() {
    auto& state = SystemState::getInstance();

    static uint32_t lastThermalCheck       = 0;
    const uint32_t  THERMAL_CHECK_INTERVAL = 1000;  // Check every 1 second

    uint32_t currentTime = millis();

    if (state.isReady()) {
        commander.processSerialInput();

        // Regular thermal monitoring
        if (currentTime - lastThermalCheck >= THERMAL_CHECK_INTERVAL) {
            motor.updateThermalStatus();  // Update all thermal status information

            auto status = motor.getThermalStatus();  // Get current thermal status
            state.setTemperature(status.currentTemp);

            if (status.protectionActive) {
                if (status.currentTemp >= status.maxTemp) {
                    state.setError(SystemState::ErrorCode::THERMAL_ERROR);
                    motor.disable();  // Disable motor for protection
                } else {
                    state.setError(SystemState::ErrorCode::THERMAL_WARNING);
                }
            }

            lastThermalCheck = currentTime;
        }
    }
}

/**
 * @brief Handles fatal errors by entering infinite error state
 * @param errorMsg Error message to display
 */
void handleFatalError(const char* errorMsg) {
    Serial.println(errorMsg);
    while (1) {
        delay(1000);
    }
}