#include "Helper/CommandHandler.h"
#include "Globals.h"
#include "Helper/System.h"

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
        return (motorNum >= 1 && motorNum <= System::NUM_MOTORS);
    }

    bool CommandHandler::validateCommandFormat(const String& cmd) const
    {
        if (cmd.isEmpty())
            return false;

        int spaceCount = 0;
        for (size_t i = 0; i < cmd.length(); i++)
        {
            if (cmd.charAt(i) == ' ')
                spaceCount++;
        }

        return spaceCount == 0 || spaceCount == 2;
    }

    bool CommandHandler::isValidMotorFormat(const String& motor) const
    {
        return motor.length() == 2 && motor.charAt(0) == 'm' && isDigit(motor.charAt(1));
    }

    bool CommandHandler::isValidParameterFormat(const String& param) const
    {
        return Utils::getInstance().isNumber(param);
    }

    CommandResult CommandHandler::handleInvalidCommand(const String& cmd, CommandStatus status)
    {
        String errorMessage;
        switch (status)
        {
            case CommandStatus::INVALID_COMMAND:
                errorMessage = "❌ Invalid command format. Use h/? for help";
                break;
            case CommandStatus::INVALID_MOTOR_NUMBER:
                errorMessage = "❌ Invalid motor number. Valid range: 1-" + String(System::NUM_MOTORS);
                break;
            case CommandStatus::INVALID_PARAMETER:
                errorMessage = "❌ Invalid parameter format";
                break;
            case CommandStatus::MOTOR_COMMUNICATION_FAILED:
                errorMessage = "❌ Motor communication failed";
                break;
            default:
                errorMessage = "❌ Unknown error occurred";
                break;
        }

        Logger::getInstance().log(errorMessage);
        if (!cmd.isEmpty())
        {
            Logger::getInstance().log(F(": "));
            Logger::getInstance().logln(cmd);
        }

        return CommandResult{status, errorMessage, false};
    }

    CommandResult CommandHandler::processCommand(String cmd)
    {
        if (!validateCommandFormat(cmd))
        {
            return handleInvalidCommand(cmd, CommandStatus::INVALID_COMMAND);
        }

        int    spaceIndex1 = cmd.indexOf(" ");
        int    spaceIndex2 = cmd.indexOf(" ", spaceIndex1 + 1);
        String firstWord   = cmd.substring(0, spaceIndex1);
        String secondWord  = (spaceIndex1 != -1) ? cmd.substring(spaceIndex1 + 1, spaceIndex2) : "";
        String thirdWord   = (spaceIndex2 != -1) ? cmd.substring(spaceIndex2 + 1) : "";

        const Command* command = findCommand(firstWord);
        if (!command)
        {
            return handleInvalidCommand(cmd, CommandStatus::INVALID_COMMAND);
        }

        if (secondWord.isEmpty() && thirdWord.isEmpty())
        {
            if (command->type == CommandType::HELP)
            {
                printCommandGuide();
                return CommandResult{CommandStatus::SUCCESS, "Help displayed", true};
            }
            return handleInvalidCommand(cmd, CommandStatus::INVALID_COMMAND);
        }

        if (!secondWord.isEmpty() && thirdWord.isEmpty())
        {
            return handleInvalidCommand(cmd, CommandStatus::INVALID_COMMAND);
        }

        if (!secondWord.isEmpty() && !thirdWord.isEmpty())
        {
            if (!isValidParameterFormat(secondWord))
            {
                return handleInvalidCommand(cmd, CommandStatus::INVALID_PARAMETER);
            }

            if (!isValidMotorFormat(thirdWord))
            {
                return handleInvalidCommand(cmd, CommandStatus::INVALID_MOTOR_NUMBER);
            }

            int motorNum = thirdWord.charAt(1) - '0';
            if (!validateMotorNumber(motorNum))
            {
                return handleInvalidCommand(cmd, CommandStatus::INVALID_MOTOR_NUMBER);
            }

            return executeMotorCommand(motorNum, command->type, secondWord.toFloat(), cmd);
        }

        return handleInvalidCommand(cmd, CommandStatus::INVALID_COMMAND);
    }

    CommandResult CommandHandler::executeMotorCommand(int motorNum, CommandType type, float position,
                                                      String fullCommand)
    {
        if (motorNum == 100 || motors[motorNum - 1].testCommunication(false))
        {
            switch (type)
            {
                case CommandType::MOTOR_RESET_LIMIT:
                    if (motionController[motorNum - 1].getLimitSwitch() != nullptr)
                    {
                        Logger::getInstance().log(F("Resetting limit for Motor "));
                        Logger::getInstance().log(String(motorNum));
                        Logger::getInstance().logln(F(": "));
                        motionController[motorNum - 1].getLimitSwitch()->reset();
                    }
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
                    return handleInvalidCommand(fullCommand, CommandStatus::INVALID_COMMAND);
            }
            return CommandResult{CommandStatus::SUCCESS, "Command executed successfully", true};
        }
        else
        {
            return handleInvalidCommand(fullCommand, CommandStatus::MOTOR_COMMUNICATION_FAILED);
        }
    }

    CommandResult CommandHandler::testCommand(const String& cmd)
    {
        CommandResult result = processCommand(cmd);
        Logger::getInstance().log(F("Test result: "));
        Logger::getInstance().logln(result.message);
        return result;
    }

    void CommandHandler::begin()
    {  // Create serial task
        xTaskCreate(serialTask, "SerialTask", 4096, this, 2, &serialTaskHandle);
    }

    void CommandHandler::serialTask(void* pvParameters)
    {
        CommandHandler* handler = static_cast<CommandHandler*>(pvParameters);
        while (1)
        {
            handler->processSerialInput();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    bool CommandHandler::processSerialInput()
    {
        if (Serial.available() > 0)
        {
            String command = Serial.readStringUntil('\n');
            command.trim();

            if (command.length() > 10)
            {
                Logger::getInstance().log(F("❌ Invalid command. Use h/? for help"));
                return false;
            }

            processSerialCommand(command);
            return true;
        }
        return false;
    }

    void CommandHandler::processSerialCommand(const String& command)
    {
        CommandResult result = processCommand(command);
        Logger::getInstance().logln(result.message);
    }
}  // namespace MotionSystem