#include "CommandHandler.h"

MotionSystem::CommandHandler* MotionSystem::CommandHandler::instance = nullptr;

const MotionSystem::Command MotionSystem::CommandHandler::commands[] = {
    {"Forward", "Move motor forward", CommandType::MOTOR_FORWARD, MotionSystem::Config::CommandHandler::CMD_FORWARD,
     true},
    {"Reverse", "Move motor in reverse", CommandType::MOTOR_REVERSE, MotionSystem::Config::CommandHandler::CMD_REVERSE,
     true},
    {"Stop", "Stop motor", CommandType::MOTOR_STOP, MotionSystem::Config::CommandHandler::CMD_STOP, true},
    {"Test", "Test motor communication", CommandType::DRIVER_SPI_TEST,
     MotionSystem::Config::CommandHandler::CMD_TEST_SPI, true},
    {"Info", "Show system information", CommandType::DRIVER_STATUS,
     MotionSystem::Config::CommandHandler::CMD_SHOW_STATUS, true},
    {"Print", "Show Driver Configuration", CommandType::DRIVER_CONFIG,
     MotionSystem::Config::CommandHandler::CMD_SHOW_CONFIG, true},
    {"Temp", "Show temperature", CommandType::TEMPERATURE, MotionSystem::Config::CommandHandler::CMD_SHOW_TEMP, true},
    {"Mode", "Toggle mode", CommandType::MODE_TOGGLE, MotionSystem::Config::CommandHandler::CMD_TOGGLE_MODE, true},
    {"Reset", "Reset driver", CommandType::DRIVER_RESET, MotionSystem::Config::CommandHandler::CMD_RESET, true},
    {"Help", "Show command guide", CommandType::HELP, MotionSystem::Config::CommandHandler::CMD_HELP, false},
    {"?", "Show command guide", CommandType::HELP, MotionSystem::Config::CommandHandler::CMD_HELP_ALT, false}};

const size_t MotionSystem::CommandHandler::NUM_COMMANDS = sizeof(commands) / sizeof(commands[0]);

MotionSystem::CommandHandler& MotionSystem::CommandHandler::getInstance()
{
    if (!instance)
    {
        instance = new CommandHandler();
    }

    return *instance;
}

MotionSystem::CommandHandler::CommandHandler() {}

void MotionSystem::CommandHandler::printCommandGuide()
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

const MotionSystem::Command* MotionSystem::CommandHandler::findCommand(const char* input) const
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

bool MotionSystem::CommandHandler::validateMotorNumber(int motorNum) const
{
    return (motorNum >= 1 && motorNum <= Config::TMC5160T_Driver::NUM_MOTORS);
}

void MotionSystem::CommandHandler::executeMotorCommand(int motorNum, CommandType type)
{
    if (motors[motorNum - 1].testCommunication(false))
    {
        switch (type)
        {
            case CommandType::MOTOR_FORWARD:
                Serial.print(F("Motor "));
                Serial.print(motorNum);
                Serial.println(F(" moving forward"));
                motors[motorNum - 1].moveForward();
                break;
            case CommandType::MOTOR_REVERSE:
                Serial.print(F("Motor "));
                Serial.print(motorNum);
                Serial.println(F(" moving reverse"));
                motors[motorNum - 1].moveReverse();
                break;
            case CommandType::MOTOR_STOP:
                Serial.print(F("Motor "));
                Serial.print(motorNum);
                Serial.println(F(" stopped"));
                motors[motorNum - 1].stop();
                break;
            case CommandType::DRIVER_RESET:
                Serial.print(F("Resetting driver for Motor "));
                Serial.print(motorNum);
                Serial.println(F(": "));
                motors[motorNum - 1].resetDriverState();
                break;
            case CommandType::DRIVER_SPI_TEST:
                Serial.print(F("\nMotor "));
                Serial.print(motorNum);
                Serial.print(F(" - "));
                motors[motorNum - 1].testCommunication();
                break;
            case CommandType::DRIVER_STATUS:
                Serial.print(F("\nDriver Status for Motor "));
                Serial.print(motorNum);
                Serial.println(F(": "));
                motors[motorNum - 1].printDriverStatus();
                break;
            case CommandType::DRIVER_CONFIG:
                Serial.print(F("\nDriver Configuration for Motor "));
                Serial.print(motorNum);
                Serial.println(F(": "));
                motors[motorNum - 1].printDriverConfig();
                break;
            case CommandType::TEMPERATURE:
                Serial.print(F("\nTemperature for Motor "));
                Serial.print(motorNum);
                Serial.print(F(": "));
                motors[motorNum - 1].printTemperature();
                break;
            case CommandType::MODE_TOGGLE:
                Serial.print(F("\nToggling mode for Motor "));
                Serial.print(motorNum);
                Serial.println(F(": "));
                motors[motorNum - 1].toggleStealthChop();
                break;
            case CommandType::HELP:
                printCommandGuide();
                break;
            default:
                Serial.println(F("Invalid command type ❌ "));
                break;
        }
    }

    else
    {
        Serial.print(F("\nMotor "));
        Serial.print(motorNum);
        Serial.println(F(" communication failed ❌ "));
    }
}

bool MotionSystem::CommandHandler::isValidMotorCommand(char cmd) const
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

void MotionSystem::CommandHandler::processCommand(char cmd, int motorNum)
{
    const Command* command = findCommand(&cmd);
    if (!command)
    {
        Serial.println(F("Invalid command ❌"));
        return;
    }

    if (command->requiresMotorNumber)
    {
        if (!validateMotorNumber(motorNum))
        {
            Serial.print(F("Invalid motor number (1-"));
            Serial.print(Config::TMC5160T_Driver::NUM_MOTORS);
            Serial.println(F(") ❌"));
            return;
        }

        executeMotorCommand(motorNum, command->type);
    }

    else
    {
        if (command->type == CommandType::DRIVER_STATUS)
        {
            Serial.println(F("System Information:"));
            Serial.print(F("Number of motors: "));
            Serial.println(Config::TMC5160T_Driver::NUM_MOTORS);
        }
    }
}