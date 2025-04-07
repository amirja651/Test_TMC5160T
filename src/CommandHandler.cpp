#include "CommandHandler.h"

// Initialize static members
CommandHandler* CommandHandler::instance = nullptr;

// Command definitions
const Command CommandHandler::commands[] = {
    {"Forward", "Move motor forward", CommandType::MOTOR_FORWARD, Config::CommandHandler::CMD_FORWARD, true},
    {"Reverse", "Move motor in reverse", CommandType::MOTOR_REVERSE, Config::CommandHandler::CMD_REVERSE, true},
    {"Stop", "Stop motor", CommandType::MOTOR_STOP, Config::CommandHandler::CMD_STOP, true},

    {"Test", "Test motor communication", CommandType::DRIVER_SPI_TEST, Config::CommandHandler::CMD_TEST_SPI, true},

    {"Info", "Show system information", CommandType::DRIVER_STATUS, Config::CommandHandler::CMD_SHOW_STATUS, true},
    {"Print", "Show Driver Configuration", CommandType::DRIVER_CONFIG, Config::CommandHandler::CMD_SHOW_CONFIG, true},

    {"Temp", "Show temperature", CommandType::TEMPERATURE, Config::CommandHandler::CMD_SHOW_TEMP, true},
    {"Mode", "Toggle mode", CommandType::MODE_TOGGLE, Config::CommandHandler::CMD_TOGGLE_MODE, true},

    {"Reset", "Reset driver", CommandType::DRIVER_RESET, Config::CommandHandler::CMD_RESET, true},

    {"Help", "Show command guide", CommandType::HELP, Config::CommandHandler::CMD_HELP, false},
    {"?", "Show command guide", CommandType::HELP, Config::CommandHandler::CMD_HELP_ALT, false}};

const size_t CommandHandler::NUM_COMMANDS = sizeof(commands) / sizeof(commands[0]);

/**
 * @brief Get the singleton instance of CommandHandler
 * @return Reference to the CommandHandler instance
 *
 * This method implements the singleton pattern, ensuring only one
 * instance of CommandHandler exists throughout the program's lifetime.
 */
CommandHandler& CommandHandler::getInstance()
{
    if (!instance)
    {
        instance = new CommandHandler();
    }
    return *instance;
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
 * @brief Print the command guide with all available commands
 *
 * Displays a formatted list of all available commands and their
 * descriptions. This is shown when the help command is received
 * or when an invalid command is entered.
 */
void CommandHandler::printCommandGuide()
{
    Serial.println("\n ==================== Available Commands ====================");
    Serial.println("\nMovement Commands (example: motor 1 w - mean motor 1 will move forward):");
    for (size_t i = 0; i < NUM_COMMANDS; i++)
    {
        if (commands[i].requiresMotorNumber)
        {
            Serial.print("  motor x ");
            Serial.print(commands[i].key);
            Serial.print(" - ");
            Serial.println(commands[i].description);
        }
    }
    Serial.println("\nSystem Commands:");
    for (size_t i = 0; i < NUM_COMMANDS; i++)
    {
        if (!commands[i].requiresMotorNumber)
        {
            Serial.print("  ");
            Serial.print(commands[i].key);
            Serial.print(" - ");
            Serial.println(commands[i].description);
        }
    }
    Serial.println("\n ==================== End of Command Guide ====================\n> ");
}

const Command* CommandHandler::findCommand(const char* input) const
{
    for (size_t i = 0; i < NUM_COMMANDS; i++)
    {
        if (input[0] == commands[i].key)
        {
            return &commands[i];
        }
    }
    return nullptr;
}

bool CommandHandler::validateMotorNumber(int motorNum) const
{
    return (motorNum >= 1 && motorNum <= Config::TMC5160T_Driver::NUM_MOTORS);
}

void CommandHandler::executeMotorCommand(int motorNum, CommandType type)
{
    if (motors[motorNum - 1].testCommunication(false))
    {
        switch (type)
        {
            case CommandType::MOTOR_FORWARD:
                Serial.print("Motor ");
                Serial.print(motorNum);
                Serial.println(" moving forward");
                motors[motorNum - 1].moveForward();
                break;
            case CommandType::MOTOR_REVERSE:
                Serial.print("Motor ");
                Serial.print(motorNum);
                Serial.println(" moving reverse");
                motors[motorNum - 1].moveReverse();
                break;
            case CommandType::MOTOR_STOP:
                Serial.print("Motor ");
                Serial.print(motorNum);
                Serial.println(" stopped");
                motors[motorNum - 1].stop();
                break;

            case CommandType::DRIVER_RESET:
                Serial.print("Resetting driver for Motor ");
                Serial.print(motorNum);
                Serial.println(": ");
                motors[motorNum - 1].resetDriverState();
                break;

            case CommandType::DRIVER_SPI_TEST:
                Serial.print("\nMotor ");
                Serial.print(motorNum);
                Serial.print(" - ");
                motors[motorNum - 1].testCommunication();
                break;

            case CommandType::DRIVER_STATUS:
                Serial.print("\nDriver Status for Motor ");
                Serial.print(motorNum);
                Serial.println(": ");
                motors[motorNum - 1].printDriverStatus();
                break;

            case CommandType::DRIVER_CONFIG:
                Serial.print("\nDriver Configuration for Motor ");
                Serial.print(motorNum);
                Serial.println(": ");
                motors[motorNum - 1].printDriverConfig();
                break;

            case CommandType::TEMPERATURE:
                Serial.print("\nTemperature for Motor ");
                Serial.print(motorNum);
                Serial.print(": ");
                motors[motorNum - 1].printTemperature();
                break;

            case CommandType::MODE_TOGGLE:
                Serial.print("\nToggling mode for Motor ");
                Serial.print(motorNum);
                Serial.println(": ");
                motors[motorNum - 1].toggleStealthChop();
                break;

            case CommandType::HELP:
                printCommandGuide();
                break;

            default:
                Serial.println("Invalid command type ❌ ");
                break;
        }
    }
    else
    {
        Serial.print("\nMotor ");
        Serial.print(motorNum);
        Serial.println(" communication failed ❌ ");
    }
}

bool CommandHandler::isValidMotorCommand(char cmd) const
{
    for (size_t i = 0; i < NUM_COMMANDS; i++)
    {
        if (commands[i].requiresMotorNumber && commands[i].key == cmd)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief Process incoming command character
 * @param cmd The command character to process
 *
 * This method handles all user input commands by mapping them to
 * appropriate motor control operations.
 */
void CommandHandler::processCommand(char cmd, int motorNum)
{
    const Command* command = findCommand(&cmd);

    if (!command)
    {
        Serial.println("Invalid command ❌");
        return;
    }

    if (command->requiresMotorNumber)
    {
        if (!validateMotorNumber(motorNum))
        {
            Serial.print("Invalid motor number (1-");
            Serial.print(Config::TMC5160T_Driver::NUM_MOTORS);
            Serial.println(") ❌");
            return;
        }
        executeMotorCommand(motorNum, command->type);
    }
    else
    {
        // Handle system commands here
        if (command->type == CommandType::DRIVER_STATUS)
        {
            Serial.println("System Information:");
            Serial.print("Number of motors: ");
            Serial.println(Config::TMC5160T_Driver::NUM_MOTORS);
            // Add more system info as needed
        }
    }
}