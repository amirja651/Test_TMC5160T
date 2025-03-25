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
    Serial.println("\nMovement:");
    Serial.println("  w/s | x -> Move Forward/Reverse | Stop");
    Serial.println("\nCurrent Control:");
    Serial.println("  q/e -> Increase/Decrease Run Current");
    Serial.println("  a/d -> Increase/Decrease Hold Current");
    Serial.println("  c -> Show Current Settings");
    Serial.println("\nSpeed Control:");
    Serial.println("  r/f -> Increase/Decrease Speed");
    Serial.println("  u/j -> Increase/Decrease Acceleration");
    Serial.println("  v -> Show Speed Settings");
    Serial.println("\nStatus & Diagnostics:");
    Serial.println("  i -> Show Detailed Driver Status");
    Serial.println("  p -> Show Driver Configuration");
    Serial.println("  m -> Show Temperature");
    Serial.println("\nSystem:");
    Serial.println("  z -> Reset Driver");
    Serial.println("  t -> Test SPI");
    Serial.println("  h or ? -> Show this guide");
    Serial.println("Motor Mode:");
    Serial.println("  n -> Toggle StealthChop/SpreadCycle Mode");
}

void CommandHandler::processCommand(char cmd) {
    switch (cmd) {
        case 'w':
            MotorController::getInstance().moveForward();
            Serial.println("Moving Forward");
            break;
        case 's':
            MotorController::getInstance().moveReverse();
            Serial.println("Moving Reverse");
            break;
        case 'x':
            MotorController::getInstance().stop();
            Serial.println("Stopped");
            break;
        case 'z':  // Reset driver
            resetDriver();
            break;
        case 't':  // Test SPI
            retestSPI();
            break;
        case 'q':  // Decrease run current
            MotorController::getInstance().increaseRunCurrent();
            break;
        case 'e':  // Decrease run current
            MotorController::getInstance().decreaseRunCurrent();
            break;
        case 'a':  // Increase hold current
            MotorController::getInstance().increaseHoldCurrent();
            break;
        case 'd':  // Decrease hold current
            MotorController::getInstance().decreaseHoldCurrent();
            break;
        case 'c':  // Print current settings
            Serial.print("Run current: ");
            Serial.print(MotorController::getInstance().getRunCurrent());
            Serial.print("mA, Hold current: ");
            Serial.print(MotorController::getInstance().getHoldCurrent());
            Serial.println("mA");
            break;
        case 'r':  // Increase speed
            MotorController::getInstance().increaseSpeed();
            break;
        case 'f':  // Decrease speed
            MotorController::getInstance().decreaseSpeed();
            break;
        case 'u':  // Increase acceleration
            MotorController::getInstance().increaseAcceleration();
            break;
        case 'j':  // Decrease acceleration
            MotorController::getInstance().decreaseAcceleration();
            break;
        case 'v':  // Print speed settings
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
        case 'm':
            MotorController::getInstance().printTemperature();
            break;
        case 'n':  // Toggle StealthChop/SpreadCycle mode
            MotorController::getInstance().toggleStealthChop();
            break;
        case '?':
        case 'h':  // Show command guide
            printCommandGuide();
            break;
        default:
            Serial.println("Invalid command. Type 'h' or '?' for help.");
            break;
    }
}

void CommandHandler::printStatus() {
    uint32_t status = MotorController::getInstance().getDriverStatus();
    Serial.print("DRV_STATUS: 0x");
    Serial.println(status, HEX);
}