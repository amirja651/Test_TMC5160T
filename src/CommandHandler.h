#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>
#include "MotorController.h"
#include "SPIManager.h"

class CommandHandler {
public:
    static CommandHandler& getInstance();
    void                   processCommand(char cmd);
    void                   printStatus();
    void                   printCommandGuide();

private:
    CommandHandler();
    CommandHandler(const CommandHandler&)            = delete;
    CommandHandler& operator=(const CommandHandler&) = delete;

    void resetDriver();
    void retestSPI();
};

#endif