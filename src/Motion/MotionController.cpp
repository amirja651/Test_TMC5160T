#include "Motion/MotionController.h"

namespace MotionSystem
{
    MotionController::MotionController(EncoderInterface* encoder, TmcController* motor, PIDController* pidController,
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

    void MotionController::begin()
    {
        encoder->begin();
        motor->begin();
        pidController->init();
        limitSwitch->init();
        lastStepTime                           = esp_timer_get_time();
        Types::EncoderPosition initialPosition = encoder->readPosition();
        statusReporter->setAbsoluteZeroPosition(initialPosition);
        statusReporter->setRelativeZeroPosition(initialPosition);
        pidController->setTargetPosition(initialPosition);
        Logger::getInstance().logln(F("Motion controller initialized"));
    }

    void MotionController::moveToPosition(Types::MicronPosition positionMicrons)
    {
        if (positionMicrons < -Config::System::REL_TRAVEL_LIMIT_MICRONS ||
            positionMicrons > Config::System::REL_TRAVEL_LIMIT_MICRONS)
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage, positionMicrons, Config::System::REL_TRAVEL_LIMIT_MM);
            Logger::getInstance().logln(buffer);
            return;
        }

        Types::EncoderPosition targetPosition = statusReporter->getRelativeZeroPosition() +
                                                MotionSystem::Utils::getInstance().micronsToEncCounts(positionMicrons);
        pidController->setTargetPosition(targetPosition);
        snprintf_P(buffer, sizeof(buffer), movingMessage, positionMicrons, (long)targetPosition);
        Logger::getInstance().logln(buffer);
    }

    void MotionController::moveRelative(Types::MicronPosition distanceMicrons)
    {
        Types::MicronPosition currentRelPos = statusReporter->getRelativePosition();
        Types::MicronPosition newRelPos     = currentRelPos + distanceMicrons;
        if (newRelPos < -Config::System::REL_TRAVEL_LIMIT_MICRONS ||
            newRelPos > Config::System::REL_TRAVEL_LIMIT_MICRONS)
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage3, newRelPos, Config::System::REL_TRAVEL_LIMIT_MM);
            Logger::getInstance().logln(buffer);
            return;
        }

        Types::EncoderPosition currentPosition = encoder->readPosition();
        Types::EncoderPosition targetPosition =
            currentPosition + MotionSystem::Utils::getInstance().micronsToEncCounts(distanceMicrons);
        pidController->setTargetPosition(targetPosition);
        snprintf_P(buffer, sizeof(buffer), movingMessage3, distanceMicrons,
                   distanceMicrons / Config::System::PIXEL_SIZE, targetPosition);
        Logger::getInstance().logln(buffer);
    }

    bool MotionController::waitForMotionComplete(float toleranceMicrons, uint32_t timeoutMs)
    {
        Types::EncoderPosition toleranceCounts =
            MotionSystem::Utils::getInstance().micronsToEncCounts(toleranceMicrons);
        uint32_t startTime = millis();
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

    void MotionController::resetRelativeZero()
    {
        Types::EncoderPosition currentPosition = encoder->readPosition();
        statusReporter->setRelativeZeroPosition(currentPosition);
        Logger::getInstance().logln(F("Relative zero position reset at current position"));
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
                Logger::getInstance().logln(F("EMERGENCY STOP: Limit switch triggered!"));
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
                    controller->motor->step();
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
        Logger::getInstance().logln(F("Motion control task started"));
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
