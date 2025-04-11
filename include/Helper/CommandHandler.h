#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "Config.h"
#include "Globals.h"
#include "Helper/Logger.h"
#include "Helper/Utils.h"
#include "Motion/MotionGlobals.h"

namespace MotionSystem
{
    /**
     * @brief Command execution status codes
     */
    enum class CommandStatus
    {
        SUCCESS,
        INVALID_COMMAND,
        INVALID_MOTOR_NUMBER,
        INVALID_PARAMETER,
        MOTOR_COMMUNICATION_FAILED,
        COMMAND_EXECUTION_FAILED
    };

    /**
     * @brief Command type enumeration
     */
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

    /**
     * @brief Command structure containing command information
     */
    struct Command
    {
        const char* name;         ///< Command name
        const char* description;  ///< Command description
        CommandType type;         ///< Command type
        const char* key;          ///< Command key/identifier
    };

    /**
     * @brief Command execution result structure
     */
    struct CommandResult
    {
        CommandStatus status;   ///< Command execution status
        String        message;  ///< Result message
        bool          success;  ///< Execution success flag
    };

    /**
     * @brief Command handler for motion control system
     *
     * This class manages command processing and execution for the motion control system.
     * It supports motor control commands, system status commands, and configuration commands.
     *
     * Usage example:
     * @code
     * CommandHandler::getInstance().processCommand("MOV 100 m1"); // Move motor 1 by 100 steps
     * @endcode
     */
    class CommandHandler
    {
    public:
        /**
         * @brief Get the singleton instance
         * @return Reference to the CommandHandler instance
         */
        static CommandHandler& getInstance()
        {
            static CommandHandler instance;  // Guaranteed to be created only once
            return instance;
        }

        // Delete copy constructor and assignment operator
        CommandHandler(const CommandHandler&)            = delete;
        CommandHandler& operator=(const CommandHandler&) = delete;

        /**
         * @brief Process a command string
         * @param cmd The command string to process
         * @return CommandResult containing execution status and message
         */
        CommandResult processCommand(String cmd);

        /**
         * @brief Print system status
         */
        void printStatus();

        /**
         * @brief Print command guide
         */
        void printCommandGuide();

        /**
         * @brief Find a command by its key
         * @param cmd The command key to search for
         * @return Pointer to the Command if found, nullptr otherwise
         */
        const Command* findCommand(String cmd) const;

        /**
         * @brief Validate motor number
         * @param motorNum The motor number to validate
         * @return true if valid, false otherwise
         */
        bool validateMotorNumber(int motorNum) const;

        /**
         * @brief Execute a motor command
         * @param motorNum The motor number
         * @param type The command type
         * @param position The position parameter
         * @param fullCommand The full command string (for error reporting)
         * @return CommandResult containing execution status
         */
        CommandResult executeMotorCommand(int motorNum, CommandType type, float position, String fullCommand = "");

        /**
         * @brief Check if a command is valid
         * @param cmd The command to check
         * @return true if valid, false otherwise
         */
        bool isValidMotorCommand(String cmd) const;

        /**
         * @brief Test command execution
         * @param cmd The command to test
         * @return CommandResult containing test results
         */
        CommandResult testCommand(const String& cmd);

        /**
         * @brief Validate command format
         * @param cmd The command to validate
         * @return true if format is valid, false otherwise
         */
        bool validateCommandFormat(const String& cmd) const;

        /**
         * @brief Initialize serial communication
         * @param baudRate Serial baud rate
         */
        void beginSerial(uint32_t baudRate = Config::System::SERIAL_BAUD_RATE);

        /**
         * @brief Process serial input
         * @return true if command was processed, false otherwise
         */
        bool processSerialInput();

        /**
         * @brief Get the serial task handle
         * @return TaskHandle_t for the serial task
         */
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