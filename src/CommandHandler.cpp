#include "CommandHandler.h"

CommandHandler& CommandHandler::getInstance() {
    static CommandHandler instance;
    return instance;
}

CommandHandler::CommandHandler() {}

void CommandHandler::resetDriver() {
    Serial.println("Resetting driver...");
    MotorController::getInstance().begin();
    Serial.println("Driver reset complete");
}

void CommandHandler::retestSPI() {
    Serial.println("Testing SPI communication...");
    SPIManager::getInstance().testCommunication();
    Serial.println("SPI test complete");
}

void CommandHandler::printCommandGuide() {
    Serial.println("\nCommand Guide:");
    Serial.println("Movement:");
    Serial.println("  f -> Move Forward | r -> Move Reverse | s -> Stop");
    Serial.println("Current Control:");
    Serial.println("  +/- -> Increase/Decrease Run Current");
    Serial.println("  h/l -> Increase/Decrease Hold Current");
    Serial.println("  c -> Show Current Settings");
    Serial.println("Speed Control:");
    Serial.println("  v/b -> Increase/Decrease Speed");
    Serial.println("  a/z -> Increase/Decrease Acceleration");
    Serial.println("  d -> Show Speed Settings");
    Serial.println("Status & Diagnostics:");
    Serial.println("  i -> Show Detailed Driver Status");
    Serial.println("  p -> Show Driver Configuration");
    Serial.println("  m -> Show Temperature");
    Serial.println("System:");
    Serial.println("  x -> Reset Driver");
    Serial.println("  t -> Test SPI");
    Serial.println("  q -> Quit");
    Serial.println("  h -> Show this guide\n");
}

void CommandHandler::processCommand(char cmd) {
    switch (cmd) {
        case 'f':
            MotorController::getInstance().moveForward();
            Serial.println("Moving Forward");
            break;
        case 'r':
            MotorController::getInstance().moveReverse();
            Serial.println("Moving Reverse");
            break;
        case 's':
            MotorController::getInstance().stop();
            Serial.println("Stopped");
            break;
        case 'x':  // Reset driver
            resetDriver();
            break;
        case 't':  // Test SPI
            retestSPI();
            break;
        case '+':  // Increase run current
            MotorController::getInstance().increaseRunCurrent();
            break;
        case '-':  // Decrease run current
            MotorController::getInstance().decreaseRunCurrent();
            break;
        case 'h':  // Increase hold current
            MotorController::getInstance().increaseHoldCurrent();
            break;
        case 'l':  // Decrease hold current
            MotorController::getInstance().decreaseHoldCurrent();
            break;
        case 'c':  // Print current settings
            Serial.print("Run current: ");
            Serial.print(MotorController::getInstance().getRunCurrent());
            Serial.print("mA, Hold current: ");
            Serial.print(MotorController::getInstance().getHoldCurrent());
            Serial.println("mA");
            break;
        case 'v':  // Increase speed
            MotorController::getInstance().increaseSpeed();
            break;
        case 'b':  // Decrease speed
            MotorController::getInstance().decreaseSpeed();
            break;
        case 'a':  // Increase acceleration
            MotorController::getInstance().increaseAcceleration();
            break;
        case 'z':  // Decrease acceleration
            MotorController::getInstance().decreaseAcceleration();
            break;
        case 'd':  // Print speed settings
            Serial.print("Speed: ");
            Serial.print(MotorController::getInstance().getSpeed());
            Serial.print(" steps/sec, Acceleration: ");
            Serial.print(MotorController::getInstance().getAcceleration());
            Serial.println(" steps/sec²");
            break;
        case 'i':  // Show detailed driver status
            MotorController::getInstance().printDriverStatus();
            break;
        case 'p':  // Show driver configuration
            MotorController::getInstance().printDriverConfig();
            break;
        case 'q':
            MotorController::getInstance().stop();
            Serial.println("Exiting...");
            break;
        case 'm':
            MotorController::getInstance().printTemperature();
            break;
        default:
            Serial.println("Invalid command. Type 'h' for help.");
            break;
    }
}

void CommandHandler::printStatus() {
    uint32_t status = MotorController::getInstance().getDriverStatus();
    Serial.print("DRV_STATUS: 0x");
    Serial.println(status, HEX);
}