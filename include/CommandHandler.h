#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "Config.h"
#include "MotorInstances.h"

// Command type definitions
enum class CommandType
{
    MOTOR_MOVE,
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

// Command structure
struct Command
{
    const char* name;
    const char* description;
    CommandType type;
    char        key;
    bool        requiresMotorNumber;
};

/**
 * @brief Command Handler class for processing user input commands
 *
 * This class implements a singleton pattern to handle user commands
 * for controlling the TMC5160T motor driver. It processes single-character
 * commands and executes corresponding motor control operations.
 */
class CommandHandler
{
public:
    /**
     * @brief Get the singleton instance of CommandHandler
     * @return Reference to the CommandHandler instance
     */
    static CommandHandler& getInstance();

    /**
     * @brief Process a single character command
     * @param cmd The command character to process
     */
    void processCommand(char cmd, int motorNum = -1);

    /**
     * @brief Print the current driver status register value
     */
    void printStatus();

    /**
     * @brief Print the command guide with all available commands
     */
    void printCommandGuide();

    // Helper functions
    const Command* findCommand(const char* input) const;
    bool           validateMotorNumber(int motorNum) const;
    void           executeMotorCommand(int motorNum, CommandType type);

    // New helper function for command validation
    bool isValidMotorCommand(char cmd) const;

private:
    CommandHandler();
    CommandHandler(const CommandHandler&)            = delete;
    CommandHandler& operator=(const CommandHandler&) = delete;

    static CommandHandler* instance;

    // Command definitions array
    static const Command commands[];
    static const size_t  NUM_COMMANDS;
};

#endif