#include <Arduino.h>
#include "CommandHandler.h"
#include "MAE3Encoder.h"
#include "MotorController.h"
#include "SPIManager.h"
#include "config.h"

// Initialize encoder on pin 2 (adjust as needed)
MAE3Encoder encoder(ESP32Pins::LeftSide::GPIO36, true);  // true for 12-bit mode
uint16_t    lastPosition = 0;                            // Store last position for comparison

/**
 * @brief Initial setup of the system
 * Configures serial communication, SPI interface, and motor controller
 */
void setup() {
    // Initialize serial communication for command interface
    Serial.begin(Config::System::SERIAL_BAUD_RATE);
    delay(Config::System::STARTUP_DELAY_MS);  // Startup delay for stability

    // Initialize encoder
    encoder.begin();
    Serial.println("Encoder initialized");

    // Initialize and test SPI communication
    SPIManager::getInstance().begin();
    SPIManager::getInstance().testCommunication();
    Serial.println("SPI Test completed");

    // Initialize motor controller and display command guide
    MotorController::getInstance().begin();
    CommandHandler::getInstance().printCommandGuide();
}

/**
 * @brief Main program loop
 * Handles serial command processing and motor controller updates
 */
void loop() {
    // Process incoming serial commands if available
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        CommandHandler::getInstance().processCommand(cmd);
    }

    // Read encoder position and angle
    uint16_t position = encoder.readPosition();

    // Only print if position has changed
    if (position != lastPosition) {
        float angle = encoder.readAngle();
        Serial.print("Position: ");
        Serial.print(position);
        Serial.print(" Angle: ");
        Serial.println(angle);
        lastPosition = position;
    }

    // Update motor controller state
    MotorController::getInstance().update();

    delay(10);  // Reduced delay for better responsiveness
}