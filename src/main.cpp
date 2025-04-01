#include <Arduino.h>
#include "CommandHandler.h"
#include "MultiMotorController.h"
#include "config.h"

/**
 * @brief Initial setup of the system
 * Configures serial communication, SPI interface, and motor controller
 */
void setup() {
    Serial.begin(115200);
    delay(1000);  // Wait for serial to initialize

    // Initialize controller for 4 motors
    MultiMotorController::getInstance().begin();

    // Control individual motors
    MultiMotorController::getInstance().moveForward(0);  // Move first motor forward
    MultiMotorController::getInstance().moveReverse(1);  // Move second motor in reverse

    // Display command guide
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
    MultiMotorController::getInstance().update();
}