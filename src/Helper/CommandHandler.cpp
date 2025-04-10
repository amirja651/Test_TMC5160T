#include "Helper/CommandHandler.h"
#include "Globals.h"
#include "Helper/Utils.h"
#include "Motion/MotionGlobals.h"
namespace MotionSystem
{
    const String CommandKey::CMD_MOVE        = "MOV";
    const String CommandKey::CMD_FORWARD     = "FWD";
    const String CommandKey::CMD_REVERSE     = "REV";
    const String CommandKey::CMD_STOP        = "STP";
    const String CommandKey::CMD_RESET       = "RST";
    const String CommandKey::CMD_TEST_SPI    = "TST";
    const String CommandKey::CMD_SHOW_STATUS = "STA";
    const String CommandKey::CMD_SHOW_CONFIG = "CFG";
    const String CommandKey::CMD_TOGGLE_MODE = "MOD";
    const String CommandKey::CMD_HELP        = "HLP";
    const String CommandKey::CMD_HELP_ALT    = "?";

    const Command CommandHandler::commands[] = {
        {"Move", "Move motor", CommandType::MOTOR_FORWARD, CommandKey::CMD_MOVE},
        {"Forward", "Move motor forward", CommandType::MOTOR_FORWARD, CommandKey::CMD_FORWARD},
        {"Reverse", "Move motor in reverse", CommandType::MOTOR_REVERSE, CommandKey::CMD_REVERSE},
        {"Stop", "Stop motor", CommandType::MOTOR_STOP, CommandKey::CMD_STOP},
        {"Test", "Test motor communication", CommandType::DRIVER_SPI_TEST, CommandKey::CMD_TEST_SPI},
        {"Info", "Show system information", CommandType::DRIVER_STATUS, CommandKey::CMD_SHOW_STATUS},
        {"Print", "Show Driver Configuration", CommandType::DRIVER_CONFIG, CommandKey::CMD_SHOW_CONFIG},
        {"Mode", "Toggle mode", CommandType::MODE_TOGGLE, CommandKey::CMD_TOGGLE_MODE},
        {"Reset", "Reset driver", CommandType::DRIVER_RESET, CommandKey::CMD_RESET},
        {"Help", "Show command guide", CommandType::HELP, CommandKey::CMD_HELP},
        {"?", "Show command guide", CommandType::HELP, CommandKey::CMD_HELP_ALT}};

    const size_t CommandHandler::NUM_COMMANDS = sizeof(commands) / sizeof(commands[0]);

    CommandHandler* CommandHandler::instance = nullptr;

    CommandHandler& CommandHandler::getInstance()
    {
        if (!instance)
        {
            instance = new CommandHandler();
        }

        return *instance;
    }

    CommandHandler::CommandHandler() {}

    void CommandHandler::printCommandGuide()
    {
        Serial.println("\n ==================== Available Commands ====================");
        Serial.println("\nMovement Commands (example: motor 1 w - mean motor 1 will move forward):");
        for (size_t i = 0; i < NUM_COMMANDS; i++)
        {
            Serial.print("  motor x ");
            Serial.print(commands[i].key);
            Serial.print(" - ");
            Serial.println(commands[i].description);
        }

        Serial.println("\nSystem Commands:");
        for (size_t i = 0; i < NUM_COMMANDS; i++)
        {
            Serial.print("  ");
            Serial.print(commands[i].key);
            Serial.print(" - ");
            Serial.println(commands[i].description);
        }

        Serial.println("\n ==================== End of Command Guide ====================\n> ");
    }

    const Command* CommandHandler::findCommand(String cmd) const
    {
        for (size_t i = 0; i < NUM_COMMANDS; i++)
        {
            if (cmd.equalsIgnoreCase(commands[i].key))
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
        /*if (command.startsWith("MOVE "))
        {
            float position = command.substring(5).toFloat();
            moveToPosition(position);
        }

        else if (command.startsWith("REL "))
        {
            float distance = command.substring(4).toFloat();
            moveRelative(distance);
        }

        else if (command == "STATUS")
        {
            statusReporter->printStatusUpdate(true);
        }

        else if (command == "RESET_LIMIT")
        {
            limitSwitch->reset();
            Serial.println(F("Limit switch flag reset"));
        }

        else if (command == "RESET_POS")
        {
            resetRelativeZero();
        }

        else
        {
            Serial.println(F("Unknown command"));
        }*/

        if (motors[motorNum - 1].testCommunication(false))
        {
            switch (type)
            {
                case CommandType::MOTOR_MOVE:
                    Serial.print(F("Motor "));
                    Serial.print(motorNum);
                    Serial.println(F(" moving"));
                    // motionController.moveToPosition(position);

                    break;
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

    bool CommandHandler::isValidMotorCommand(String cmd) const
    {
        for (size_t i = 0; i < NUM_COMMANDS; i++)
        {
            if (cmd.equalsIgnoreCase(commands[i].key))
            {
                return true;
            }
        }

        return false;
    }

    void CommandHandler::processCommand(String cmd)
    {
        int            spaceIndex1 = cmd.indexOf(" ");
        int            spaceIndex2 = cmd.indexOf(" ", spaceIndex1 + 1);
        String         firstWord   = cmd.substring(0, spaceIndex1);
        String         secondWord  = (spaceIndex1 != -1) ? cmd.substring(spaceIndex1 + 1, spaceIndex2) : "";
        String         thirdWord   = (spaceIndex2 != -1) ? cmd.substring(spaceIndex2 + 1) : "";
        const Command* command     = findCommand(firstWord);

        if (secondWord == "" && thirdWord == "")
        {
            if (!command)
            {
                Serial.println(F("❌ Invalid command ["));
                Serial.print(cmd);
                Serial.println(F("]. Use h/? for help"));
                return;
            }

            if (command->type == CommandType::HELP)
            {
                printCommandGuide();
            }
        }

        else if (secondWord != "" && thirdWord == "")
        {
            Serial.println(F("❌ Invalid command ["));
            Serial.print(cmd);
            Serial.println(F("]. Use h/? for help"));
            return;
        }

        else if (secondWord != "" && thirdWord != "")
        {
            if (Utils::getInstance().isNumber(secondWord))
            {
                if (thirdWord.length() == 2 && thirdWord.charAt(0) == 'm' && isDigit(thirdWord.charAt(1)))
                {
                    int motorNum = thirdWord.charAt(1) - '0';

                    if (!validateMotorNumber(motorNum))
                    {
                        Serial.print(F(" ❌ Invalid motor number: "));
                        Serial.print(motorNum);
                        return;
                    }

                    executeMotorCommand(motorNum, command->type);
                }
                else
                {
                    Serial.println(F("❌ Invalid command ["));
                    Serial.print(cmd);
                    Serial.println(F("]. Use h/? for help"));
                    return;
                }
            }
            else
            {
                Serial.println(F("❌ Invalid command ["));
                Serial.print(cmd);
                Serial.println(F("]. Use h/? for help"));
                return;
            }
        }

        else
        {
            Serial.println("Invalid command.");
        }
    }
}  // namespace MotionSystem