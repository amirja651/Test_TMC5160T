#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "Config.h"
#include "Globals.h"
#include "Helper/Logger.h"
#include "Helper/Utils.h"
#include "Motion/MotionGlobals.h"

namespace MotionSystem
{
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
        const char* name;
        const char* description;
        CommandType type;
        const char* key;
    };

    class CommandHandler
    {
    public:
        static CommandHandler& getInstance();
        void                   processCommand(String cmd);
        void                   printStatus();
        void                   printCommandGuide();
        const Command*         findCommand(String cmd) const;
        bool                   validateMotorNumber(int motorNum) const;
        void executeMotorCommand(int motorNum, CommandType type, float position, String fullCommand = "");
        bool isValidMotorCommand(String cmd) const;

    private:
        CommandHandler();
        CommandHandler(const CommandHandler&)                   = delete;
        CommandHandler&        operator=(const CommandHandler&) = delete;
        static CommandHandler* instance;
        static const Command   commands[];
        static const size_t    NUM_COMMANDS;
    };
}  // namespace MotionSystem

#endif