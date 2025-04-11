#include "Helper/CommandHandler.h"

namespace MotionSystem
{
    const Command CommandHandler::commands[] = {
        {.name = "Move", .description = "Move motor", .type = CommandType::MOTOR_FORWARD, .key = "MOV"},
        {.name = "Forward", .description = "Move motor forward", .type = CommandType::MOTOR_FORWARD, .key = "FWD"},
        {.name = "Reverse", .description = "Move motor in reverse", .type = CommandType::MOTOR_REVERSE, .key = "REV"},
        {.name = "Stop", .description = "Stop motor", .type = CommandType::MOTOR_STOP, .key = "STP"},
        {.name = "Test", .description = "Test motor communication", .type = CommandType::DRIVER_SPI_TEST, .key = "TST"},
        {.name = "Info", .description = "Show system information", .type = CommandType::DRIVER_STATUS, .key = "STA"},
        {.name = "Print", .description = "Show Driver Configuration", .type = CommandType::DRIVER_CONFIG, .key = "CFG"},
        {.name = "Mode", .description = "Toggle mode", .type = CommandType::MODE_TOGGLE, .key = "MOD"},
        {.name = "Reset", .description = "Reset driver", .type = CommandType::DRIVER_RESET, .key = "RST"},
        {.name = "Help", .description = "Show command guide", .type = CommandType::HELP, .key = "HLP"},
        {.name = "?", .description = "Show command guide", .type = CommandType::HELP, .key = "?"}};

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

    void CommandHandler::executeMotorCommand(int motorNum, CommandType type, float position, String fullCommand)
    {
        if (motorNum == 100 || motors[motorNum - 1].testCommunication(false))
        {
            switch (type)
            {
                case CommandType::MOTOR_RESET_LIMIT:
                    Logger::getInstance().log(F("Resetting limit for Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(": "));
                    limitSwitch.reset();
                    break;
                case CommandType::MOTOR_RESET_POS:
                    Logger::getInstance().log(F("Resetting position for Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(": "));
                    motionController[motorNum - 1].resetRelativeZero();
                    break;
                case CommandType::MOTOR_MOVE:
                    Logger::getInstance().log(F("Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(" moving"));
                    motionController[motorNum - 1].moveToPosition(position);
                    break;
                case CommandType::MOTOR_RELATIVE_MOVE:
                    Logger::getInstance().log(F("Motor "));
                    Logger::getInstance().log(String(motorNum));
                    Logger::getInstance().logln(F(" moving relative"));
                    motionController[motorNum - 1].moveRelative(position);
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
                case CommandType::STATUS_REPORT:
                    // statusReporter->printStatusUpdate(true);
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
                case CommandType::INVALID:
                default:
                    Logger::getInstance().log(F("❌ Invalid command. Use h/? for help: "));
                    Logger::getInstance().logln(fullCommand);
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
                executeMotorCommand(100, CommandType::INVALID, 0, cmd);
                return;
            }

            if (command->type == CommandType::HELP)
            {
                printCommandGuide();
            }
        }

        else if (secondWord != "" && thirdWord == "")
        {
            executeMotorCommand(100, CommandType::INVALID, 0, cmd);
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
                        executeMotorCommand(100, CommandType::INVALID, 0, cmd);
                        return;
                    }

                    executeMotorCommand(motorNum, command->type, secondWord.toFloat());
                }
                else
                {
                    executeMotorCommand(100, CommandType::INVALID, 0, cmd);
                    return;
                }
            }
            else
            {
                executeMotorCommand(100, CommandType::INVALID, 0, cmd);
                return;
            }
        }
        else
        {
            executeMotorCommand(100, CommandType::INVALID, 0, cmd);
        }
    }
}  // namespace MotionSystem