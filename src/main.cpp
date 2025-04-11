#include "Globals.h"
#include "Helper/CommandHandler.h"
#include "Helper/Logger.h"

using namespace MotionSystem;

auto& commandHandler = MotionSystem::CommandHandler::getInstance();

void setup()
{
    commandHandler.beginSerial();

    Logger::getInstance().begin();

    Logger::getInstance().logln(
        F("\n ==================== Initializing High Precision Motion Control System ===================="));

    initializeMotors();
    initializePWMEncoders();
    initializeMotionSystem();

    CommandHandler::getInstance().printCommandGuide();
}

void loop()
{
    delay(10);
}