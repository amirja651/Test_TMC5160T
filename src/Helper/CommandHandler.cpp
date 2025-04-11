#include "Helper/CommandHandler.h"

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
        Logger::getInstance().logln("\n ==================== Available Commands ====================");
        Logger::getInstance().logln("\nMovement Commands (example: motor 1 w - mean motor 1 will move forward):");
        for (size_t i = 0; i < NUM_COMMANDS; i++)
        {
            Logger::getInstance().log("  motor x ");
            Logger::getInstance().log(commands[i].key);
            Logger::getInstance().log(" - ");
            Logger::getInstance().logln(commands[i].description);
        }

        Logger::getInstance().logln("\nSystem Commands:");
        for (size_t i = 0; i < NUM_COMMANDS; i++)
        {
            Logger::getInstance().log("  ");
            Logger::getInstance().log(commands[i].key);
            Logger::getInstance().log(" - ");
            Logger::getInstance().logln(commands[i].description);
        }

        Logger::getInstance().logln("\n ==================== End of Command Guide ====================\n> ");
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
            Logger::getInstance().logln(F("Limit switch flag reset"));
        }

        else if (command == "RESET_POS")
        {
            resetRelativeZero();
        }

        else
        {
            Logger::getInstance().logln(F("Unknown command"));
        }*/

        if (motors[motorNum - 1].testCommunication(false))
        {
            switch (type)
            {
                case CommandType::MOTOR_MOVE:
                    Logger::getInstance().log(F("Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(" moving"));
                    // motionController.moveToPosition(position);

                    break;
                case CommandType::MOTOR_FORWARD:
                    Logger::getInstance().log(F("Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(" moving forward"));
                    motors[motorNum - 1].moveForward();
                    break;
                case CommandType::MOTOR_REVERSE:
                    Logger::getInstance().log(F("Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(" moving reverse"));
                    motors[motorNum - 1].moveReverse();
                    break;
                case CommandType::MOTOR_STOP:
                    Logger::getInstance().log(F("Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(" stopped"));
                    motors[motorNum - 1].stop();
                    break;
                case CommandType::DRIVER_RESET:
                    Logger::getInstance().log(F("Resetting driver for Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(": "));
                    motors[motorNum - 1].resetDriverState();
                    break;
                case CommandType::DRIVER_SPI_TEST:
                    Logger::getInstance().log(F("\nMotor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().log(F(" - "));
                    motors[motorNum - 1].testCommunication();
                    break;
                case CommandType::DRIVER_STATUS:
                    Logger::getInstance().log(F("\nDriver Status for Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(": "));
                    motors[motorNum - 1].printDriverStatus();
                    break;
                case CommandType::DRIVER_CONFIG:
                    Logger::getInstance().log(F("\nDriver Configuration for Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(": "));
                    motors[motorNum - 1].printDriverConfig();
                    break;
                case CommandType::TEMPERATURE:
                    Logger::getInstance().log(F("\nTemperature for Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().log(F(": "));
                    motors[motorNum - 1].printTemperature();
                    break;
                case CommandType::MODE_TOGGLE:
                    Logger::getInstance().log(F("\nToggling mode for Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(": "));
                    motors[motorNum - 1].toggleStealthChop();
                    break;
                case CommandType::HELP:
                    printCommandGuide();
                    break;
                default:
                    Logger::getInstance().logln(F("Invalid command type ❌ "));
                    break;
            }
        }

        else
        {
            Logger::getInstance().log(F("\nMotor "));
            Logger::getInstance().log(String(motorNum));
            Logger::getInstance().logln(F(" communication failed ❌ "));
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
                Logger::getInstance().logln(F("❌ Invalid command ["));
                Logger::getInstance().log(cmd);
                Logger::getInstance().logln(F("]. Use h/? for help"));
                return;
            }

            if (command->type == CommandType::HELP)
            {
                printCommandGuide();
            }
        }

        else if (secondWord != "" && thirdWord == "")
        {
            Logger::getInstance().logln(F("❌ Invalid command ["));
            Logger::getInstance().log(cmd);
            Logger::getInstance().logln(F("]. Use h/? for help"));
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
                        Logger::getInstance().log(F(" ❌ Invalid motor number: "));
                        Logger::getInstance().log(String(motorNum));
                        return;
                    }

                    executeMotorCommand(motorNum, command->type);
                }
                else
                {
                    Logger::getInstance().logln(F("❌ Invalid command ["));
                    Logger::getInstance().log(cmd);
                    Logger::getInstance().logln(F("]. Use h/? for help"));
                    return;
                }
            }
            else
            {
                Logger::getInstance().logln(F("❌ Invalid command ["));
                Logger::getInstance().log(cmd);
                Logger::getInstance().logln(F("]. Use h/? for help"));
                return;
            }
        }

        else
        {
            Logger::getInstance().logln("Invalid command.");
        }
    }
}  // namespace MotionSystem