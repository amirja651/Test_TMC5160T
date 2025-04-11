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
        MOTOR_MOVE,
        MOTOR_FORWARD,
        MOTOR_REVERSE,
        MOTOR_STOP,
        DRIVER_SPI_TEST,
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
        const char*  name;
        const char*  description;
        CommandType  type;
        const String key;
    };

    namespace CommandKey
    {
        extern const String CMD_MOVE;
        extern const String CMD_FORWARD;
        extern const String CMD_REVERSE;
        extern const String CMD_STOP;
        extern const String CMD_RESET;
        extern const String CMD_TEST_SPI;
        extern const String CMD_SHOW_STATUS;
        extern const String CMD_SHOW_CONFIG;
        extern const String CMD_TOGGLE_MODE;
        extern const String CMD_HELP;
        extern const String CMD_HELP_ALT;
    }  // namespace CommandKey

    class CommandHandler
    {
    public:
        static CommandHandler& getInstance();
        void                   processCommand(String cmd);
        void                   printStatus();
        void                   printCommandGuide();
        const Command*         findCommand(String cmd) const;
        bool                   validateMotorNumber(int motorNum) const;
        void                   executeMotorCommand(int motorNum, CommandType type);
        bool                   isValidMotorCommand(String cmd) const;

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