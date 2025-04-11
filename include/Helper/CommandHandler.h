#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "Helper/Logger.h"
#include "Helper/System.h"
#include "Helper/Utils.h"

namespace MotionSystem
{
    enum class CommandStatus
    {
        SUCCESS,
        INVALID_COMMAND,
        INVALID_MOTOR_NUMBER,
        INVALID_PARAMETER,
        MOTOR_COMMUNICATION_FAILED,
        COMMAND_EXECUTION_FAILED
    };

    enum class CommandType
    {
        MOTOR_RESET_LIMIT,
        MOTOR_RESET_POS,
        MOTOR_MOVE,
        MOTOR_RELATIVE_MOVE,
        MOTOR_FORWARD,
        MOTOR_REVERSE,
        MOTOR_STOP,
        DRIVER_SPI_TEST,
        STATUS_REPORT,
        DRIVER_STATUS,
        DRIVER_CONFIG,
        TEMPERATURE,
        MODE_TOGGLE,
        DRIVER_RESET,
        HELP,
        INVALID
    };

    struct Command
    {
        const char* name;         ///< Command name
        const char* description;  ///< Command description
        CommandType type;         ///< Command type
        const char* key;          ///< Command key/identifier
    };

    struct CommandResult
    {
        CommandStatus status;   ///< Command execution status
        String        message;  ///< Result message
        bool          success;  ///< Execution success flag
    };

    class CommandHandler
    {
    public:
        static CommandHandler& getInstance()
        {
            static CommandHandler instance;  // Guaranteed to be created only once
            return instance;
        }

        // Delete copy constructor and assignment operator
        CommandHandler(const CommandHandler&)            = delete;
        CommandHandler& operator=(const CommandHandler&) = delete;

        CommandResult processCommand(String cmd);

        void printStatus();

        void printCommandGuide();

        const Command* findCommand(String cmd) const;

        bool validateMotorNumber(int motorNum) const;

        CommandResult executeMotorCommand(int motorNum, CommandType type, float position, String fullCommand = "");

        bool isValidMotorCommand(String cmd) const;

        CommandResult testCommand(const String& cmd);

        bool validateCommandFormat(const String& cmd) const;

        void beginSerial(uint32_t baudRate = System::SERIAL_BAUD_RATE);

        bool processSerialInput();

        TaskHandle_t getSerialTaskHandle() const
        {
            return serialTaskHandle;
        }

    private:
        // Private constructor
        CommandHandler() = default;

        // Internal helper methods
        CommandResult handleInvalidCommand(const String& cmd, CommandStatus status);
        bool          isValidMotorFormat(const String& motor) const;
        bool          isValidParameterFormat(const String& param) const;

        static const Command commands[];
        static const size_t  NUM_COMMANDS;

        // Serial handling members
        TaskHandle_t serialTaskHandle = nullptr;
        static void  serialTask(void* pvParameters);
        void         processSerialCommand(const String& command);
    };
}  // namespace MotionSystem

#endif