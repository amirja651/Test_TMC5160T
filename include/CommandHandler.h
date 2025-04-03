#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>
#include "MotorController.h"

/**
 * @brief Command Handler class for processing user input commands
 *
 * This class implements a singleton pattern to handle user commands
 * for controlling the TMC5160T motor driver. It processes single-character
 * commands and executes corresponding motor control operations.
 */
class CommandHandler {
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
    void processCommand(char cmd);

    /**
     * @brief Print the current driver status register value
     */
    void printStatus();

    /**
     * @brief Print the command guide with all available commands
     */
    void printCommandGuide();

private:
    CommandHandler();
    CommandHandler(const CommandHandler&)            = delete;
    CommandHandler& operator=(const CommandHandler&) = delete;

    /**
     * @brief Reset the motor driver and reinitialize it
     */
    void resetDriver();

    /**
     * @brief Test SPI communication with the motor driver
     */
    void retestSPI();
};

#endif