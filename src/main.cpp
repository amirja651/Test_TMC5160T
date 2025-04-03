#include <Arduino.h>
#include "CommandHandler.h"
#include "MotorController.h"
#include "config.h"
#include "motor_instances.h"

/**
 * @brief Initial setup of the system
 * Configures serial communication, SPI interface, and motor controller
 */
void setup() {
    // Initialize serial communication for command interface
    Serial.begin(Config::System::SERIAL_BAUD_RATE);
    delay(Config::System::STARTUP_DELAY_MS);  // Startup delay for stability
    while (!Serial) {
        delay(10);
    }
    Serial.println("Initializing motor controllers...");

    // Initialize each motor controller
    motor1.begin();

    // Initialize and test SPI communication
    motor1.testCommunication();

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
    motor1.update();

    delay(10);  // Reduced delay for better responsiveness
}