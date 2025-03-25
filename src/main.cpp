#include <Arduino.h>
#include "CommandHandler.h"
#include "MotorController.h"
#include "SPIManager.h"

void setup() {
    Serial.begin(115200);
    delay(1000);  // Startup delay for stability

    SPIManager::getInstance().begin();
    SPIManager::getInstance().testCommunication();
    Serial.println("SPI Test completed");

    MotorController::getInstance().begin();
    CommandHandler::getInstance().printCommandGuide();
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        CommandHandler::getInstance().processCommand(cmd);
    }

    MotorController::getInstance().update();
}