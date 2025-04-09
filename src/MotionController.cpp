#include "MotionController.h"
#include "esp_timer.h"

namespace MotionSystem
{

    MotionController::MotionController(EncoderInterface* encoder, StepperMotor* motor, PIDController* pidController,
                                       LimitSwitch* limitSwitch, StatusReporter* statusReporter)
        : encoder(encoder),
          motor(motor),
          pidController(pidController),
          limitSwitch(limitSwitch),
          statusReporter(statusReporter),
          currentSpeed(0),
          lastStepTime(0),
          taskHandle(nullptr)
    {
    }

    MotionController::~MotionController()
    {
        if (taskHandle != nullptr)
        {
            vTaskDelete(taskHandle);
            taskHandle = nullptr;
        }
    }

    void MotionController::init()
    {
        // Initialize components
        encoder->begin();
        motor->init();
        pidController->init();
        limitSwitch->init();

        // Initialize timings
        lastStepTime = esp_timer_get_time();

        // Initialize relative and absolute zeros
        Types::EncoderPosition initialPosition = encoder->readPosition();
        statusReporter->setAbsoluteZeroPosition(initialPosition);
        statusReporter->setRelativeZeroPosition(initialPosition);

        // Set initial target to current position
        pidController->setTargetPosition(initialPosition);

        Serial.println(F("Motion controller initialized"));
    }

    void MotionController::moveToPosition(Types::MicronPosition positionMicrons, bool calibration)
    {
        // Check if within relative travel limits
        if (!calibration && (positionMicrons < -Config::System::REL_TRAVEL_LIMIT_MICRONS ||
                             positionMicrons > Config::System::REL_TRAVEL_LIMIT_MICRONS))
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage, positionMicrons, Config::System::REL_TRAVEL_LIMIT_MM);
            Serial.println(buffer);
            return;
        }

        // Convert microns to encoder counts and adjust for relative zero
        Types::EncoderPosition targetPosition =
            statusReporter->getRelativeZeroPosition() + encoder->micronsToEncCounts(positionMicrons);

        // Set the target position in PID controller
        pidController->setTargetPosition(targetPosition);

        snprintf_P(buffer, sizeof(buffer), movingMessage, positionMicrons, (long)targetPosition);
        Serial.println(buffer);
    }

    void MotionController::moveToPositionPixels(Types::PixelPosition positionPixels)
    {
        Types::MicronPosition positionMicrons = pixelsToMicrons(positionPixels);

        // Check if within relative travel limits
        if (positionMicrons < -Config::System::REL_TRAVEL_LIMIT_MICRONS ||
            positionMicrons > Config::System::REL_TRAVEL_LIMIT_MICRONS)
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage2, positionPixels, positionMicrons,
                       Config::System::REL_TRAVEL_LIMIT_MM);
            Serial.println(buffer);
            return;
        }

        Types::EncoderPosition targetPosition =
            statusReporter->getRelativeZeroPosition() + encoder->micronsToEncCounts(positionMicrons);

        // Set the target position in PID controller
        pidController->setTargetPosition(targetPosition);

        snprintf_P(buffer, sizeof(buffer), movingMessage2, positionPixels, positionMicrons, targetPosition);
        Serial.println(buffer);
    }

    void MotionController::moveRelative(Types::MicronPosition distanceMicrons)
    {
        // Check if move would exceed relative travel limits
        Types::MicronPosition currentRelPos = statusReporter->getRelativePosition();
        Types::MicronPosition newRelPos     = currentRelPos + distanceMicrons;

        if (newRelPos < -Config::System::REL_TRAVEL_LIMIT_MICRONS ||
            newRelPos > Config::System::REL_TRAVEL_LIMIT_MICRONS)
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage3, newRelPos, Config::System::REL_TRAVEL_LIMIT_MM);
            Serial.println(buffer);
            return;
        }

        Types::EncoderPosition currentPosition = encoder->readPosition();
        Types::EncoderPosition targetPosition  = currentPosition + encoder->micronsToEncCounts(distanceMicrons);

        // Set the target position in PID controller
        pidController->setTargetPosition(targetPosition);

        snprintf_P(buffer, sizeof(buffer), movingMessage3, distanceMicrons,
                   distanceMicrons / Config::System::PIXEL_SIZE, targetPosition);
        Serial.println(buffer);
    }

    void MotionController::moveRelativePixels(Types::PixelPosition distancePixels)
    {
        Types::MicronPosition distanceMicrons = pixelsToMicrons(distancePixels);

        // Check if move would exceed relative travel limits
        Types::MicronPosition currentRelPos = statusReporter->getRelativePosition();
        Types::MicronPosition newRelPos     = currentRelPos + distanceMicrons;

        if (newRelPos < -Config::System::REL_TRAVEL_LIMIT_MICRONS ||
            newRelPos > Config::System::REL_TRAVEL_LIMIT_MICRONS)
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage4, newRelPos, Config::System::REL_TRAVEL_LIMIT_MM);
            Serial.println(buffer);
            return;
        }

        Types::EncoderPosition currentPosition = encoder->readPosition();
        Types::EncoderPosition targetPosition  = currentPosition + encoder->micronsToEncCounts(distanceMicrons);

        // Set the target position in PID controller
        pidController->setTargetPosition(targetPosition);

        snprintf_P(buffer, sizeof(buffer), movingMessage4, distancePixels, distanceMicrons, targetPosition);
        Serial.println(buffer);
    }

    bool MotionController::waitForMotionComplete(float toleranceMicrons, uint32_t timeoutMs)
    {
        Types::EncoderPosition toleranceCounts = encoder->micronsToEncCounts(toleranceMicrons);
        uint32_t               startTime       = millis();

        while (millis() - startTime < timeoutMs)
        {
            Types::EncoderPosition currentPosition = encoder->readPosition();
            Types::EncoderPosition positionError   = abs(pidController->getTargetPosition() - currentPosition);

            if (positionError <= toleranceCounts && abs(currentSpeed) < 10)
            {
                return true;  // Motion complete
            }

            delay(10);  // Small delay to prevent CPU hogging
        }

        return false;  // Timeout occurred
    }

    void MotionController::calibrateSystem()
    {
        Serial.println(F("Starting system calibration..."));

        // Reset limit switch flag
        limitSwitch->reset();

        // First move away from the limit switch if already triggered
        if (limitSwitch->isTriggered())
        {
            Serial.println(F("Limit switch already triggered. Moving away..."));
            // Move in the positive direction (away from limit switch)
            moveRelative(100);  // Move 100 microns away
            waitForMotionComplete(0.1, 5000);

            // Check if we're clear of the limit switch
            if (limitSwitch->isTriggered())
            {
                Serial.println(F("Error: Could not move away from limit switch!"));
                return;
            }
        }

        // Now find the minimum position by moving toward the limit switch
        Serial.println(F("Finding minimum position (home)..."));
        moveToPosition(-99999, true);  // Move to a large negative value, bypassing limit checks

        // Wait for limit switch to trigger or timeout
        unsigned long startTime = millis();
        while (!limitSwitch->isTriggered() && (millis() - startTime < 30000))
        {
            delay(10);
        }

        if (!limitSwitch->isTriggered())
        {
            Serial.println(F("Calibration failed: Limit switch not triggered!"));
            return;
        }

        // Stop motion
        currentSpeed = 0;
        delay(500);  // Allow system to settle

        // Move away from limit switch
        Serial.println(F("Moving away from limit switch..."));
        moveRelative(50);  // Move 50 microns away
        waitForMotionComplete(0.1, 5000);

        // Define this position as our absolute zero
        Types::EncoderPosition position = encoder->readPosition();
        statusReporter->setAbsoluteZeroPosition(position);

        // Also set as relative zero by default
        statusReporter->setRelativeZeroPosition(position);

        // Reset target to current position
        pidController->setTargetPosition(position);

        // Reset limit flag
        limitSwitch->reset();

        Serial.println(F("System calibrated. Absolute zero set at current position."));
    }

    void MotionController::resetRelativeZero()
    {
        Types::EncoderPosition currentPosition = encoder->readPosition();
        statusReporter->setRelativeZeroPosition(currentPosition);
        Serial.println(F("Relative zero position reset at current position"));
    }

    Types::MicronPosition MotionController::pixelsToMicrons(Types::PixelPosition pixels)
    {
        return pixels * Config::System::PIXEL_SIZE;
    }

    void MotionController::processCommands()
    {
        if (Serial.available() > 0)
        {
            String command = Serial.readStringUntil('\n');
            command.trim();
            command.toUpperCase();

            // Parse command
            if (command.startsWith("MOVE "))
            {
                float position = command.substring(5).toFloat();
                moveToPosition(position);
            }
            else if (command.startsWith("MOVEPX "))
            {
                float positionPixels = command.substring(7).toFloat();
                moveToPositionPixels(positionPixels);
            }
            else if (command.startsWith("REL "))
            {
                float distance = command.substring(4).toFloat();
                moveRelative(distance);
            }
            else if (command.startsWith("RELPX "))
            {
                float distancePixels = command.substring(6).toFloat();
                moveRelativePixels(distancePixels);
            }
            else if (command == "HOME")
            {
                calibrateSystem();
            }
            else if (command == "STATUS")
            {
                statusReporter->printStatusUpdate(true);
            }
            else if (command == "RESET_LIMIT")
            {
                limitSwitch->reset();
                Serial.println(F("Limit switch flag reset"));
            }
            else if (command == "RESET_POS")
            {
                resetRelativeZero();
            }
            else
            {
                Serial.println(F("Unknown command"));
            }
        }
    }

    void MotionController::motionTask(void* parameter)
    {
        MotionController* controller = static_cast<MotionController*>(parameter);

        // Initialize timing
        controller->lastStepTime = esp_timer_get_time();

        while (true)
        {
            // Check for emergency stop condition
            if (controller->limitSwitch->isEmergencyStop())
            {
                controller->currentSpeed = 0;
                controller->limitSwitch->setEmergencyStop(false);  // Reset flag
                Serial.println(F("EMERGENCY STOP: Limit switch triggered!"));
                vTaskDelay(100);  // Give time for other tasks
                continue;
            }

            // Calculate desired speed from PID output
            int32_t pidOutput    = controller->pidController->update();
            float   desiredSpeed = constrain(pidOutput, -Config::Motion::MAX_SPEED, Config::Motion::MAX_SPEED);

            // Apply acceleration limits
            float maxSpeedChange = Config::Motion::ACCELERATION / Config::Motion::PID_UPDATE_FREQ;
            if (desiredSpeed > controller->currentSpeed + maxSpeedChange)
            {
                controller->currentSpeed += maxSpeedChange;
            }
            else if (desiredSpeed < controller->currentSpeed - maxSpeedChange)
            {
                controller->currentSpeed -= maxSpeedChange;
            }
            else
            {
                controller->currentSpeed = desiredSpeed;
            }

            // Update status reporter with current speed
            controller->statusReporter->setCurrentSpeed(controller->currentSpeed);

            // Set direction based on speed sign
            bool direction = controller->currentSpeed >= 0;

            // Safety check - don't move if limit switch is triggered
            if (controller->limitSwitch->isTriggered())
            {
                // Only allow movement away from the limit switch
                // Since the switch is at the minimum position (opposite end),
                // only allow positive movement (away from the switch)
                if (!direction)
                {                                  // Moving toward limit switch (negative direction)
                    controller->currentSpeed = 0;  // Stop motion in this direction
                    continue;
                }
            }

            controller->motor->setDirection(direction);

            // Calculate step interval
            uint32_t stepInterval = controller->motor->calculateStepInterval(controller->currentSpeed);

            // Generate step if needed and enough time has passed
            if (stepInterval > 0)
            {
                uint64_t now = esp_timer_get_time();
                if (now - controller->lastStepTime >= stepInterval)
                {
                    controller->motor->generateStep();
                    controller->lastStepTime = now;
                }
            }

            // Yield to other tasks
            vTaskDelay(1);
        }
    }

    void MotionController::startTask()
    {
        xTaskCreatePinnedToCore(motionTask, "Motion Control", Config::Tasks::MOTION_TASK_STACK_SIZE, this,
                                Config::Tasks::MOTION_TASK_PRIORITY, &taskHandle, Config::Tasks::MOTION_TASK_CORE);

        Serial.println(F("Motion control task started"));
    }

    void MotionController::setCurrentSpeed(Types::Speed speed)
    {
        currentSpeed = speed;
    }

    Types::Speed MotionController::getCurrentSpeed() const
    {
        return currentSpeed;
    }

}  // namespace MotionSystem

// End of code