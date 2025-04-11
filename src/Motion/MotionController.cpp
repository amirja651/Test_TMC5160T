#include "Motion/MotionController.h"
#include "Helper/System.h"
#include "Helper/Tasks.h"
#include "Helper/Utils.h"
#include "esp_timer.h"

namespace MotionSystem
{
    using namespace MotionSystem::Types;

    MotionController::MotionController(EncoderInterface* encoder, TmcController* motor, PIDController* pidController,
                                       LimitSwitch* limitSwitch)
        : encoder(encoder),
          motor(motor),
          pidController(pidController),
          limitSwitch(limitSwitch),
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
        lastStepTime                    = esp_timer_get_time();
        EncoderPosition initialPosition = encoder->readPosition();
        setAbsoluteZeroPosition(initialPosition);
        setRelativeZeroPosition(initialPosition);
        pidController->setTargetPosition(initialPosition);
        Logger::getInstance().logln(F("Motion controller initialized"));
    }

    void MotionController::moveToPosition(MicronPosition positionMicrons)
    {
        if (positionMicrons < -System::REL_TRAVEL_LIMIT_MICRONS || positionMicrons > System::REL_TRAVEL_LIMIT_MICRONS)
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage, positionMicrons, System::REL_TRAVEL_LIMIT_MM);
            Logger::getInstance().logln(buffer);
            return;
        }

        EncoderPosition targetPosition =
            getRelativeZeroPosition() + MotionSystem::Utils::getInstance().micronsToEncCounts(positionMicrons);
        pidController->setTargetPosition(targetPosition);
        snprintf_P(buffer, sizeof(buffer), movingMessage, positionMicrons, (long)targetPosition);
        Logger::getInstance().logln(buffer);
    }

    void MotionController::moveRelative(MicronPosition distanceMicrons)
    {
        MicronPosition currentRelPos = getRelativePosition();
        MicronPosition newRelPos     = currentRelPos + distanceMicrons;
        if (newRelPos < -System::REL_TRAVEL_LIMIT_MICRONS || newRelPos > System::REL_TRAVEL_LIMIT_MICRONS)
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage3, newRelPos, System::REL_TRAVEL_LIMIT_MM);
            Logger::getInstance().logln(buffer);
            return;
        }

        EncoderPosition currentPosition = encoder->readPosition();
        EncoderPosition targetPosition =
            currentPosition + MotionSystem::Utils::getInstance().micronsToEncCounts(distanceMicrons);
        pidController->setTargetPosition(targetPosition);
        snprintf_P(buffer, sizeof(buffer), movingMessage3, distanceMicrons, distanceMicrons / System::PIXEL_SIZE,
                   targetPosition);
        Logger::getInstance().logln(buffer);
    }

    bool MotionController::waitForMotionComplete(float toleranceMicrons, uint32_t timeoutMs)
    {
        EncoderPosition toleranceCounts = MotionSystem::Utils::getInstance().micronsToEncCounts(toleranceMicrons);
        uint32_t        startTime       = millis();
        while (millis() - startTime < timeoutMs)
        {
            EncoderPosition currentPosition = encoder->readPosition();
            EncoderPosition positionError   = abs(pidController->getTargetPosition() - currentPosition);
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
        EncoderPosition currentPosition = encoder->readPosition();
        setRelativeZeroPosition(currentPosition);
        Logger::getInstance().logln(F("Relative zero position reset at current position"));
    }

    void MotionController::motionTask(void* parameter)
    {
        MotionController* controller = static_cast<MotionController*>(parameter);
        controller->lastStepTime     = esp_timer_get_time();

        while (true)
        {
            Types::MicronPosition relPosition = controller->getRelativePosition();
            Types::MicronPosition relTarget   = Utils::getInstance().countsToMicrons(
                controller->pidController->getTargetPosition() - controller->relativeZeroPosition);

            if (abs(relPosition - relTarget) > 0.2)
            {
                controller->printStatusUpdate();
            }

            if (controller->limitSwitch->isEmergencyStop())
            {
                controller->currentSpeed = 0;
                controller->limitSwitch->setEmergencyStop(false);  // Reset flag
                Logger::getInstance().logln(F("EMERGENCY STOP: Limit switch triggered!"));
                vTaskDelay(100);  // Give time for other tasks
                continue;
            }

            int32_t pidOutput      = controller->pidController->update();
            float   desiredSpeed   = constrain(pidOutput, -Motion::MAX_SPEED, Motion::MAX_SPEED);
            float   maxSpeedChange = Motion::ACCELERATION / Motion::MOTION_UPDATE_FREQ;

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

            controller->setCurrentSpeed(controller->currentSpeed);
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
        xTaskCreatePinnedToCore(motionTask, "Motion Control", Tasks::MOTION_TASK_STACK_SIZE, this,
                                Tasks::MOTION_TASK_PRIORITY, &taskHandle, Tasks::MOTION_TASK_CORE);
        Logger::getInstance().logln(F("Motion control task started"));
    }

    void MotionController::setCurrentSpeed(Speed speed)
    {
        currentSpeed = speed;
    }

    Speed MotionController::getCurrentSpeed() const
    {
        return currentSpeed;
    }

    void MotionController::printStatusUpdate(bool showStatus)
    {
        Types::MicronPosition relPosition = getRelativePosition();
        Types::MicronPosition absPosition = getAbsolutePosition();
        Types::MicronPosition relTarget =
            Utils::getInstance().countsToMicrons(pidController->getTargetPosition() - relativeZeroPosition);

        float error            = relTarget - relPosition;
        float motorFrequency   = abs(currentSpeed);  // Steps per second
        float relTravelPercent = (relPosition / System::REL_TRAVEL_LIMIT_MICRONS) * 100;

        if (motorFrequency > 10 || showStatus)
        {
            Logger::getInstance().logf("POS(rel): %.3f µm (%.1f%% of ±%.1f mm), TARGET: %.3f µm, ERROR: %.3f µm\n",
                                       relPosition, abs(relTravelPercent), System::REL_TRAVEL_LIMIT_MM, relTarget,
                                       error);

            Logger::getInstance().logf("POS(abs): %.3f µm, Travel: %.1f%% of %.1f mm\n", absPosition,
                                       (absPosition / System::TOTAL_TRAVEL_MICRONS) * 100, System::TOTAL_TRAVEL_MM);

            Logger::getInstance().logf(
                "Motor Freq: %.1f Hz, Speed: %.3f mm/s, Limit Switch: %s\n", motorFrequency,
                (motorFrequency / (System::STEPS_PER_REV * System::MICROSTEPS)) * System::LEAD_SCREW_PITCH,

                limitSwitch->isTriggered() ? "TRIGGERED" : "clear");
            Logger::getInstance().logln(F("-------------------------------"));
        }
    }

    void MotionController::setAbsoluteZeroPosition(Types::EncoderPosition position)
    {
        absoluteZeroPosition = position;
    }

    void MotionController::setRelativeZeroPosition(Types::EncoderPosition position)
    {
        relativeZeroPosition = position;
    }

    Types::MicronPosition MotionController::getAbsolutePosition()
    {
        Types::EncoderPosition currentPosition = encoder->readPosition();
        return Utils::getInstance().countsToMicrons(currentPosition - absoluteZeroPosition);
    }

    Types::MicronPosition MotionController::getRelativePosition()
    {
        Types::EncoderPosition currentPosition = encoder->readPosition();
        return Utils::getInstance().countsToMicrons(currentPosition - relativeZeroPosition);
    }

    Types::EncoderPosition MotionController::getRelativeZeroPosition() const
    {
        return relativeZeroPosition;
    }

}  // namespace MotionSystem
