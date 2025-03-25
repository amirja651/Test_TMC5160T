#include <Arduino.h>
#include "CommandHandler.h"
#include "MotorController.h"
#include "SPIManager.h"
#include "config.h"

/**
 * @brief Initial setup of the system
 * Configures serial communication, SPI interface, and motor controller
 */
void setup() {
    // Initialize serial communication for command interface
    Serial.begin(Config::System::SERIAL_BAUD_RATE);
    delay(Config::System::STARTUP_DELAY_MS);  // Startup delay for stability

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

    // Update motor controller state
    MotorController::getInstance().update();
}