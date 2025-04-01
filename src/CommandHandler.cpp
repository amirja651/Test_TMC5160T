#include "CommandHandler.h"
#include "MultiMotorController.h"
#include "config.h"

/**
 * @brief Get the singleton instance of CommandHandler
 * @return Reference to the CommandHandler instance
 *
 * This method implements the singleton pattern, ensuring only one
 * instance of CommandHandler exists throughout the program's lifetime.
 */
CommandHandler& CommandHandler::getInstance() {
    static CommandHandler instance;
    return instance;
}

/**
 * @brief Constructor for CommandHandler
 *
 * Private constructor to enforce singleton pattern.
 * No initialization is needed as this is handled by the
 * MotorController and SPIManager classes.
 */
CommandHandler::CommandHandler() {}

/**
 * @brief Reset the motor driver and reinitialize it
 *
 * This method performs a complete reset of the TMC5160T driver
 * by reinitializing the MotorController. This is useful for
 * recovering from error states or applying new configurations.
 */
void CommandHandler::resetDriver() {
    Serial.println("Resetting driver...");
    MultiMotorController::getInstance().begin();
    Serial.println("Driver reset complete");
}

/**
 * @brief Test SPI communication with the motor driver
 *
 * This method verifies the SPI communication with the TMC5160T
 * driver by sending a test pattern and checking the response.
 * It's useful for diagnosing communication issues.
 */
void CommandHandler::retestSPI() {
    Serial.println("Testing SPI communication...");
    MultiMotorController::getInstance().testCommunication();
    Serial.println("SPI test complete");
}

/**
 * @brief Print the command guide with all available commands
 *
 * Displays a formatted list of all available commands and their
 * descriptions. This is shown when the help command is received
 * or when an invalid command is entered.
 */
void CommandHandler::printCommandGuide() {
    using namespace Config::CommandHandler;

    Serial.println("\nCommand Guide:");
    Serial.println("\nMovement:");
    Serial.println("  " + String(CMD_FORWARD) + "/" + String(CMD_REVERSE) + " | " + String(CMD_STOP) +
                   " -> Move Forward/Reverse | Stop");
    Serial.println("\nCurrent Control:");
    Serial.println("  " + String(CMD_INC_RUN_CURRENT) + "/" + String(CMD_DEC_RUN_CURRENT) +
                   " -> Increase/Decrease Run Current");
    Serial.println("  " + String(CMD_INC_HOLD_CURRENT) + "/" + String(CMD_DEC_HOLD_CURRENT) +
                   " -> Increase/Decrease Hold Current");
    Serial.println("  " + String(CMD_SHOW_CURRENT) + " -> Show Current Settings");
    Serial.println("\nSpeed Control:");
    Serial.println("  " + String(CMD_INC_SPEED) + "/" + String(CMD_DEC_SPEED) + " -> Increase/Decrease Speed");
    Serial.println("  " + String(CMD_INC_ACCEL) + "/" + String(CMD_DEC_ACCEL) + " -> Increase/Decrease Acceleration");
    Serial.println("  " + String(CMD_SHOW_SPEED) + " -> Show Speed Settings");
    Serial.println("\nStatus & Diagnostics:");
    Serial.println("  " + String(CMD_SHOW_STATUS) + " -> Show Detailed Driver Status");
    Serial.println("  " + String(CMD_SHOW_CONFIG) + " -> Show Driver Configuration");
    Serial.println("  " + String(CMD_SHOW_TEMP) + " -> Show Temperature");
    Serial.println("\nSystem:");
    Serial.println("  " + String(CMD_RESET) + " -> Reset Driver");
    Serial.println("  " + String(CMD_TEST_SPI) + " -> Test SPI");
    Serial.println("  " + String(CMD_HELP) + " or " + String(CMD_HELP_ALT) + " -> Show this guide");
    Serial.println("\nMotor Mode:");
    Serial.println("  " + String(CMD_TOGGLE_MODE) + " -> Toggle StealthChop/SpreadCycle Mode");
}

/**
 * @brief Process incoming command character
 * @param cmd The command character to process
 *
 * This method handles all user input commands by mapping them to
 * appropriate motor control operations. It supports commands for:
 * - Movement control (forward, reverse, stop)
 * - Current control (run and hold current adjustment)
 * - Speed control (speed and acceleration adjustment)
 * - Status monitoring and diagnostics
 * - System operations (reset, SPI test)
 * - Help and configuration display
 */
void CommandHandler::processCommand(char cmd) {
    using namespace Config::CommandHandler;

    switch (cmd) {
        case CMD_FORWARD:
            MultiMotorController::getInstance().moveForward(0);
            Serial.println("Moving Forward");
            break;
        case CMD_REVERSE:
            MultiMotorController::getInstance().moveReverse(0);
            Serial.println("Moving Reverse");
            break;
        case CMD_STOP:
            MultiMotorController::getInstance().stop(0);
            Serial.println("Stopped");
            break;
        case CMD_RESET:
            resetDriver();
            break;
        case CMD_TEST_SPI:
            retestSPI();
            break;
        case CMD_INC_RUN_CURRENT:
            MultiMotorController::getInstance().increaseRunCurrent(0);
            break;
        case CMD_DEC_RUN_CURRENT:
            MultiMotorController::getInstance().decreaseRunCurrent(0);
            break;
        case CMD_INC_HOLD_CURRENT:
            MultiMotorController::getInstance().increaseHoldCurrent(0);
            break;
        case CMD_DEC_HOLD_CURRENT:
            MultiMotorController::getInstance().decreaseHoldCurrent(0);
            break;
        case CMD_SHOW_CURRENT:
            Serial.print("Run current: ");
            Serial.print(MultiMotorController::getInstance().getRunCurrent(0));
            Serial.print("mA, Hold current: ");
            Serial.print(MultiMotorController::getInstance().getHoldCurrent(0));
            Serial.println("mA");
            break;
        case CMD_INC_SPEED:
            MultiMotorController::getInstance().increaseSpeed(0);
            break;
        case CMD_DEC_SPEED:
            MultiMotorController::getInstance().decreaseSpeed(0);
            break;
        case CMD_INC_ACCEL:
            MultiMotorController::getInstance().increaseAcceleration(0);
            break;
        case CMD_DEC_ACCEL:
            MultiMotorController::getInstance().decreaseAcceleration(0);
            break;
        case CMD_SHOW_SPEED:
            Serial.print("Speed: ");
            Serial.print(MultiMotorController::getInstance().getSpeed(0));
            Serial.print(" steps/sec, Acceleration: ");
            Serial.print(MultiMotorController::getInstance().getAcceleration(0));
            Serial.println(" steps/sec²");
            break;
        case CMD_SHOW_STATUS:
            MultiMotorController::getInstance().printDriverStatus(0);
            break;
        case CMD_SHOW_CONFIG:
            MultiMotorController::getInstance().printDriverConfig(0);
            break;
        case CMD_SHOW_TEMP:
            MultiMotorController::getInstance().printTemperature(0);
            break;
        case CMD_TOGGLE_MODE:
            MultiMotorController::getInstance().toggleStealthChop(0);
            break;
        case CMD_HELP:
        case CMD_HELP_ALT:
            printCommandGuide();
            break;
        default:
            Serial.println("Invalid command. Type '" + String(CMD_HELP) + "' or '" + String(CMD_HELP_ALT) +
                           "' for help.");
            break;
    }
}

/**
 * @brief Print the current driver status register value
 *
 * Displays the raw DRV_STATUS register value from the TMC5160T
 * driver in hexadecimal format. This is useful for debugging
 * driver state and error conditions.
 */
void CommandHandler::printStatus() {
    uint32_t status = MultiMotorController::getInstance().getDriverStatus(0);
    Serial.print("DRV_STATUS: 0x");
    Serial.println(status, HEX);
}