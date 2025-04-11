#include <Arduino.h>
#include "Encoders/EncoderFactory.h"
#include "Encoders/PWMEncoder.h"
#include "Helper/Logger.h"
#include "Helper/Types.h"
#include "Motion/PIDController.h"

using namespace MotionSystem;

// Create instances with static to prevent multiple definitions
static EncoderConfig pwmEncoderConfigTest = EncoderFactory::createPWMConfig(35, 35);
static PWMEncoder    encoderTest(pwmEncoderConfigTest);
static PIDController pidControllerTest(&encoderTest);

void setup1()
{
    Serial.begin(115200);
    delay(1000);  // Wait for serial to initialize

    // Initialize the logger
    Logger::getInstance().begin();

    // Initialize encoder
    encoderTest.begin();

    // Initialize PID controller
    pidControllerTest.init();

    // Configure PID gains (optional - defaults are set in Config.h)
    pidControllerTest.setGains(1.2f, 0.15f, 0.08f);

    // Set limits (optional)
    pidControllerTest.setIntegralLimit(1000.0f);
    pidControllerTest.setOutputLimit(2000.0f);

    // Start the PID control task
    pidControllerTest.startTask();

    Logger::getInstance().logln(F("PID Controller Example Started"));
}

void loop1()
{
    static uint32_t lastStatusTime  = 0;
    static uint32_t lastMoveTime    = 0;
    const uint32_t  STATUS_INTERVAL = 1000;  // 1 second
    const uint32_t  MOVE_INTERVAL   = 5000;  // 5 seconds

    uint32_t currentTime = millis();

    // Print status periodically
    if (currentTime - lastStatusTime >= STATUS_INTERVAL)
    {
        lastStatusTime = currentTime;

        // Get current stats
        PIDStats stats;
        pidControllerTest.getStats(stats);

        // Print status information
        char statusBuffer[256];
        snprintf(statusBuffer, sizeof(statusBuffer),
                 "Status: Updates=%u, AvgTime=%.3fms, Errors=%u, MaxError=%.2f, CurrentError=%.2f, Output=%.2f",
                 stats.updateCount, stats.averageUpdateTime * 1000.0f, stats.errorCount, stats.maxError,
                 stats.currentError, stats.currentOutput);

        Logger::getInstance().logln(statusBuffer);
    }

    // Example of moving to different positions
    if (currentTime - lastMoveTime >= MOVE_INTERVAL)
    {
        lastMoveTime = currentTime;

        // Example positions in microns
        static const float positions[]   = {0.0f, 1000.0f, 2000.0f, 1000.0f, 0.0f};
        static uint8_t     positionIndex = 0;

        // Set new target position
        float targetPosition = positions[positionIndex];
        pidControllerTest.setTargetPosition(static_cast<Types::EncoderPosition>(targetPosition));

        char moveBuffer[128];
        snprintf(moveBuffer, sizeof(moveBuffer), "Moving to position: %.2f microns", targetPosition);
        Logger::getInstance().logln(moveBuffer);

        // Move to next position
        positionIndex = (positionIndex + 1) % (sizeof(positions) / sizeof(positions[0]));
    }

    // Check if PID controller is running
    if (!pidControllerTest.isRunning())
    {
        Logger::getInstance().logln(F("WARNING: PID controller is not running!"));
        delay(1000);
    }

    delay(10);  // Small delay to prevent watchdog issues
}

// Example of error handling
void handlePIDError()
{
    // Get current stats
    PIDStats stats;
    pidControllerTest.getStats(stats);

    if (stats.errorCount > 0)
    {
        char errorBuffer[128];
        snprintf(errorBuffer, sizeof(errorBuffer), "PID Error detected: %u errors, Max Error: %.2f", stats.errorCount,
                 stats.maxError);

        Logger::getInstance().logln(errorBuffer);

        // Example of error recovery
        if (stats.errorCount > 10)
        {
            Logger::getInstance().logln(F("Too many errors, resetting PID controller"));

            // Stop the current task
            pidControllerTest.stopTask();

            // Reset the controller
            pidControllerTest.init();

            // Restart the task
            pidControllerTest.startTask();
        }
    }
}