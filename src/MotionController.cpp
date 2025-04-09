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
        encoder->begin();
        motor->init();
        pidController->init();
        limitSwitch->init();
        lastStepTime                           = esp_timer_get_time();
        Types::EncoderPosition initialPosition = encoder->readPosition();
        statusReporter->setAbsoluteZeroPosition(initialPosition);
        statusReporter->setRelativeZeroPosition(initialPosition);
        pidController->setTargetPosition(initialPosition);
        Serial.println(F("Motion controller initialized"));
    }

    void MotionController::moveToPosition(Types::MicronPosition positionMicrons, bool calibration)
    {
        if (!calibration && (positionMicrons < -Config::System::REL_TRAVEL_LIMIT_MICRONS ||
                             positionMicrons > Config::System::REL_TRAVEL_LIMIT_MICRONS))
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage, positionMicrons, Config::System::REL_TRAVEL_LIMIT_MM);
            Serial.println(buffer);
            return;
        }

        Types::EncoderPosition targetPosition =
            statusReporter->getRelativeZeroPosition() + encoder->micronsToEncCounts(positionMicrons);
        pidController->setTargetPosition(targetPosition);
        snprintf_P(buffer, sizeof(buffer), movingMessage, positionMicrons, (long)targetPosition);
        Serial.println(buffer);
    }

    void MotionController::moveToPositionPixels(Types::PixelPosition positionPixels)
    {
        Types::MicronPosition positionMicrons = pixelsToMicrons(positionPixels);
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
        pidController->setTargetPosition(targetPosition);
        snprintf_P(buffer, sizeof(buffer), movingMessage2, positionPixels, positionMicrons, targetPosition);
        Serial.println(buffer);
    }

    void MotionController::moveRelative(Types::MicronPosition distanceMicrons)
    {
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
        pidController->setTargetPosition(targetPosition);
        snprintf_P(buffer, sizeof(buffer), movingMessage3, distanceMicrons,
                   distanceMicrons / Config::System::PIXEL_SIZE, targetPosition);
        Serial.println(buffer);
    }

    void MotionController::moveRelativePixels(Types::PixelPosition distancePixels)
    {
        Types::MicronPosition distanceMicrons = pixelsToMicrons(distancePixels);
        Types::MicronPosition currentRelPos   = statusReporter->getRelativePosition();
        Types::MicronPosition newRelPos       = currentRelPos + distanceMicrons;
        if (newRelPos < -Config::System::REL_TRAVEL_LIMIT_MICRONS ||
            newRelPos > Config::System::REL_TRAVEL_LIMIT_MICRONS)
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage4, newRelPos, Config::System::REL_TRAVEL_LIMIT_MM);
            Serial.println(buffer);
            return;
        }

        Types::EncoderPosition currentPosition = encoder->readPosition();
        Types::EncoderPosition targetPosition  = currentPosition + encoder->micronsToEncCounts(distanceMicrons);
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
        limitSwitch->reset();
        if (limitSwitch->isTriggered())
        {
            Serial.println(F("Limit switch already triggered. Moving away..."));
            moveRelative(100);  // Move 100 microns away
            waitForMotionComplete(0.1, 5000);
            if (limitSwitch->isTriggered())
            {
                Serial.println(F("Error: Could not move away from limit switch!"));
                return;
            }
        }

        Serial.println(F("Finding minimum position (home)..."));
        moveToPosition(-99999, true);  // Move to a large negative value, bypassing limit checks
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

        currentSpeed = 0;
        delay(500);  // Allow system to settle
        Serial.println(F("Moving away from limit switch..."));
        moveRelative(50);  // Move 50 microns away
        waitForMotionComplete(0.1, 5000);
        Types::EncoderPosition position = encoder->readPosition();
        statusReporter->setAbsoluteZeroPosition(position);
        statusReporter->setRelativeZeroPosition(position);
        pidController->setTargetPosition(position);
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
        controller->lastStepTime     = esp_timer_get_time();
        while (true)
        {
            if (controller->limitSwitch->isEmergencyStop())
            {
                controller->currentSpeed = 0;
                controller->limitSwitch->setEmergencyStop(false);  // Reset flag
                Serial.println(F("EMERGENCY STOP: Limit switch triggered!"));
                vTaskDelay(100);  // Give time for other tasks
                continue;
            }

            int32_t pidOutput      = controller->pidController->update();
            float   desiredSpeed   = constrain(pidOutput, -Config::Motion::MAX_SPEED, Config::Motion::MAX_SPEED);
            float   maxSpeedChange = Config::Motion::ACCELERATION / Config::Motion::PID_UPDATE_FREQ;
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

            controller->statusReporter->setCurrentSpeed(controller->currentSpeed);
            bool direction = controller->currentSpeed >= 0;
            if (controller->limitSwitch->isTriggered())
            {
                if (!direction)
                {                                  // Moving toward limit switch (negative direction)
                    controller->currentSpeed = 0;  // Stop motion in this direction
                    continue;
                }
            }

            controller->motor->setDirection(direction);
            uint32_t stepInterval = controller->motor->calculateStepInterval(controller->currentSpeed);
            if (stepInterval > 0)
            {
                uint64_t now = esp_timer_get_time();
                if (now - controller->lastStepTime >= stepInterval)
                {
                    controller->motor->generateStep();
                    controller->lastStepTime = now;
                }
            }

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
