#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "Config.h"

namespace MotionSystem
{
    enum class CommandType
    {
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
        const char* name;
        const char* description;
        CommandType type;
        char        key;
        bool        requiresMotorNumber;
    };
    class CommandHandler
    {
    public:
        static CommandHandler& getInstance();
        void                   processCommand(char cmd, int motorNum = -1);
        void                   printStatus();
        void                   printCommandGuide();
        const Command*         findCommand(const char* input) const;
        bool                   validateMotorNumber(int motorNum) const;
        void                   executeMotorCommand(int motorNum, CommandType type);
        bool                   isValidMotorCommand(char cmd) const;

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