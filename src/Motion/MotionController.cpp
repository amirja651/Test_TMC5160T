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
          lastEmergencyCheck(0),
          currentState(Types::MotionState::IDLE),
          taskHandle(nullptr),
          absoluteZeroPosition(0),
          relativeZeroPosition(0)
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

    bool MotionController::begin()
    {
        encoder->begin();
        motor->begin();

        // Test SPI communication first
        if (!motor->testCommunication(false))
        {
            Serial.print(motor->getInstanceName());
            Serial.println(F(" - SPI communication error detected"));
            handleError(MotionError::SPI_ERROR);
            return false;
        }

        pidController->init();

        if (limitSwitch != nullptr)
        {
            limitSwitch->init();
        }

        lastStepTime       = esp_timer_get_time();
        lastEmergencyCheck = lastStepTime;
        currentState       = MotionState::IDLE;

        EncoderPosition initialPosition = encoder->readPosition();
        setAbsoluteZeroPosition(initialPosition);
        setRelativeZeroPosition(initialPosition);
        pidController->setTargetPosition(initialPosition);

        Serial.print(motor->getInstanceName());
        Serial.println(F(" - Motion controller initialized"));

        return true;
    }

    void MotionController::handleEmergencyStop()
    {
        currentSpeed = 0;
        limitSwitch->setEmergencyStop(false);
        currentState = MotionState::ERROR;
        Serial.print(motor->getInstanceName());
        Serial.println(F(" - EMERGENCY STOP: Limit switch triggered!"));
        lastEmergencyCheck = esp_timer_get_time();
    }

    void MotionController::updateStatus()
    {
        Types::MicronPosition relPosition = getRelativePosition();
        Types::MicronPosition relTarget =
            Utils::getInstance().countsToMicrons(pidController->getTargetPosition() - relativeZeroPosition);

        if (abs(relPosition - relTarget) > 0.2)
        {
            printStatusUpdate();
        }
    }

    void MotionController::updateSpeed(float desiredSpeed, float maxSpeedChange)
    {
        if (desiredSpeed > currentSpeed + maxSpeedChange)
        {
            currentSpeed += maxSpeedChange;
            currentState = MotionState::ACCELERATING;
        }
        else if (desiredSpeed < currentSpeed - maxSpeedChange)
        {
            currentSpeed -= maxSpeedChange;
            currentState = MotionState::DECELERATING;
        }
        else
        {
            currentSpeed = desiredSpeed;
            currentState = (abs(currentSpeed) < 0.1f) ? MotionState::IDLE : MotionState::CRUISING;
        }
        setCurrentSpeed(currentSpeed);
    }

    bool MotionController::checkDirectionAndLimits()
    {
        bool newDirection = currentSpeed >= 0;

        if (limitSwitch != nullptr && limitSwitch->isTriggered() && !newDirection)
        {
            currentSpeed = 0;
            currentState = Types::MotionState::IDLE;
            return false;
        }

        // Only set direction if it has changed
        if (motor->getDirection() != newDirection)
        {
            motor->setDirection(newDirection);
        }

        return true;
    }

    void MotionController::handleStepping(uint64_t currentTime)
    {
        uint32_t stepInterval = motor->calculateStepInterval(currentSpeed);

        if (stepInterval > 0 && (currentTime - lastStepTime) >= stepInterval)
        {
            motor->update();
            lastStepTime = currentTime;
            profileMotion();
        }
    }

    uint32_t MotionController::calculateNextStepTime()
    {
        uint32_t stepInterval = motor->calculateStepInterval(currentSpeed);
        if (stepInterval == 0)
            return MIN_TASK_DELAY;

        uint64_t nextStep = lastStepTime + stepInterval;
        uint64_t now      = esp_timer_get_time();

        if (nextStep > now)
        {
            return (nextStep - now) / 1000;  // Convert to milliseconds
        }
        return MIN_TASK_DELAY;
    }

    void MotionController::handleError(MotionError error)
    {
        static uint32_t lastErrorTime = 0;
        uint32_t        currentTime   = millis();

        // Prevent error spam
        if (currentTime - lastErrorTime < 1000)  // 1 second cooldown
        {
            return;
        }
        lastErrorTime = currentTime;

        currentSpeed = 0;
        motor->enableDriver(false);
        currentState = Types::MotionState::ERROR;

        Serial.print(motor->getInstanceName());
        Serial.print(F(" - ERROR: "));

        switch (error)
        {
            case MotionError::PID_ERROR:
                Serial.println(F("PID controller error detected"));
                // Attempt to reset PID controller
                pidController->init();
                break;

            case MotionError::LIMIT_SWITCH_TRIGGERED:
                Serial.println(F("Limit switch triggered"));
                break;

            case MotionError::ENCODER_ERROR:
                Serial.println(F("Encoder error detected"));
                // Attempt to reinitialize encoder
                encoder->begin();
                break;

            case MotionError::MOTOR_ERROR:
                Serial.println(F("Motor error detected"));
                // Attempt to reset motor
                motor->resetDriverState();
                break;

            case MotionError::SPI_ERROR:
                Serial.println(F("SPI error detected"));
                // Attempt to reset motor
                motor->resetDriverState();
                break;

            default:
                Serial.println(F("Unknown error detected"));
                break;
        }

        // Attempt recovery after a short delay
        vTaskDelay(pdMS_TO_TICKS(100));
        motor->enableDriver(true);
        currentState = Types::MotionState::IDLE;
    }

    void MotionController::profileMotion()
    {
        static uint32_t stepCount       = 0;
        static uint64_t lastProfileTime = 0;

        uint64_t now = esp_timer_get_time();
        if (now - lastProfileTime >= PROFILE_INTERVAL)
        {
            float actualSpeed = (stepCount * 1000000.0f) / (now - lastProfileTime);
            Serial.printf("Motion Profile - Steps/sec: %.1f, State: %d\n", actualSpeed, static_cast<int>(currentState));
            stepCount       = 0;
            lastProfileTime = now;
        }
        stepCount++;
    }

    void MotionController::updateMotionState()
    {
        Types::MicronPosition relPosition = getRelativePosition();
        Types::MicronPosition relTarget =
            Utils::getInstance().countsToMicrons(pidController->getTargetPosition() - relativeZeroPosition);
        float positionError = abs(relTarget - relPosition);

        if (positionError < 0.1f && abs(currentSpeed) < 0.1f)
        {
            currentState = MotionState::IDLE;
        }
        else if (abs(currentSpeed) < Motion::MAX_SPEED * 0.95f)
        {
            currentState = MotionState::ACCELERATING;
        }
        else
        {
            currentState = MotionState::CRUISING;
        }
    }

    void MotionController::motionTask(void* parameter)
    {
        MotionController* controller   = static_cast<MotionController*>(parameter);
        controller->lastStepTime       = esp_timer_get_time();
        controller->lastEmergencyCheck = controller->lastStepTime;

        // Pre-calculate constants
        const float    maxSpeedChange        = Motion::ACCELERATION / Motion::MOTION_UPDATE_FREQ;
        const uint32_t statusUpdateThreshold = 1000;  // Update every 1ms
        uint32_t       lastStatusUpdate      = 0;
        uint32_t       lastPIDUpdate         = 0;
        const uint32_t PID_TIMEOUT           = 100;  // 100ms timeout for PID updates

        while (true)
        {
            uint64_t currentTime = esp_timer_get_time();

            // Emergency stop check with debounce
            if (controller->limitSwitch != nullptr && controller->limitSwitch->isEmergencyStop() &&
                (currentTime - controller->lastEmergencyCheck) > EMERGENCY_DEBOUNCE_TIME)
            {
                controller->handleEmergencyStop();
                continue;
            }

            // Check for PID timeout
            if (currentTime - lastPIDUpdate > PID_TIMEOUT * 1000)
            {
                controller->handleError(MotionError::PID_ERROR);
                lastPIDUpdate = currentTime;
                continue;
            }

            // Optimize status updates
            if ((currentTime - lastStatusUpdate) > statusUpdateThreshold)
            {
                controller->updateStatus();
                lastStatusUpdate = currentTime;
            }

            // PID control with anti-windup
            int32_t pidOutput = controller->pidController->update();
            lastPIDUpdate     = currentTime;

            // Check for PID output validity
            if (pidOutput == INT32_MIN || pidOutput == INT32_MAX)
            {
                controller->handleError(MotionError::PID_ERROR);
                continue;
            }

            float desiredSpeed = constrain(pidOutput, -Motion::MAX_SPEED, Motion::MAX_SPEED);

            // Optimized speed ramping
            controller->updateSpeed(desiredSpeed, maxSpeedChange);

            // Direction and limit switch check
            if (!controller->checkDirectionAndLimits())
            {
                continue;
            }

            // Optimized step timing
            controller->handleStepping(currentTime);

            // Update motion state
            controller->updateMotionState();

            // Adaptive task delay with minimum delay
            uint32_t nextStepTime = controller->calculateNextStepTime();
            if (nextStepTime > MIN_TASK_DELAY)
            {
                vTaskDelay(pdMS_TO_TICKS(nextStepTime));
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(MIN_TASK_DELAY));
            }
        }
    }

    void MotionController::startTask()
    {
        const UBaseType_t taskPriority = configMAX_PRIORITIES - 1;  // Highest priority
        xTaskCreatePinnedToCore(motionTask, "Motion Control", Tasks::MOTION_TASK_STACK_SIZE, this, taskPriority,
                                &taskHandle, Tasks::MOTION_TASK_CORE);
        Serial.print(motor->getInstanceName());
        Serial.println(F(" - Motion control task started"));
    }

    void MotionController::stopTask()
    {
        if (taskHandle != nullptr)
        {
            vTaskDelete(taskHandle);
            taskHandle   = nullptr;
            currentSpeed = 0;
            currentState = MotionState::IDLE;
            Serial.print(motor->getInstanceName());
            Serial.println(F(" - Motion control task stopped"));
        }
    }

    void MotionController::moveToPosition(MicronPosition positionMicrons)
    {
        if (positionMicrons < -System::REL_TRAVEL_LIMIT_MICRONS || positionMicrons > System::REL_TRAVEL_LIMIT_MICRONS)
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage, positionMicrons, System::REL_TRAVEL_LIMIT_MM);
            Serial.println(buffer);
            return;
        }

        EncoderPosition targetPosition =
            getRelativeZeroPosition() + MotionSystem::Utils::getInstance().micronsToEncCounts(positionMicrons);
        pidController->setTargetPosition(targetPosition);
        snprintf_P(buffer, sizeof(buffer), movingMessage, positionMicrons, (long)targetPosition);
        Serial.println(buffer);
    }

    void MotionController::moveRelative(MicronPosition distanceMicrons)
    {
        MicronPosition currentRelPos = getRelativePosition();
        MicronPosition newRelPos     = currentRelPos + distanceMicrons;
        if (newRelPos < -System::REL_TRAVEL_LIMIT_MICRONS || newRelPos > System::REL_TRAVEL_LIMIT_MICRONS)
        {
            snprintf_P(buffer, sizeof(buffer), errorMessage3, newRelPos, System::REL_TRAVEL_LIMIT_MM);
            Serial.println(buffer);
            return;
        }

        EncoderPosition currentPosition = encoder->readPosition();
        EncoderPosition targetPosition =
            currentPosition + MotionSystem::Utils::getInstance().micronsToEncCounts(distanceMicrons);
        pidController->setTargetPosition(targetPosition);
        snprintf_P(buffer, sizeof(buffer), movingMessage3, distanceMicrons, distanceMicrons / System::PIXEL_SIZE,
                   targetPosition);
        Serial.println(buffer);
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
        Serial.print(motor->getInstanceName());
        Serial.println(F(" - Relative zero position reset at current position"));
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
            Serial.print(motor->getInstanceName());
            Serial.println(F(": "));
            Serial.printf("POS(rel): %.3f µm (%.1f%% of ±%.1f mm), TARGET: %.3f µm, ERROR: %.3f µm\n", relPosition,
                          abs(relTravelPercent), System::REL_TRAVEL_LIMIT_MM, relTarget, error);

            Serial.printf("POS(abs): %.3f µm, Travel: %.1f%% of %.1f mm\n", absPosition,
                          (absPosition / System::TOTAL_TRAVEL_MICRONS) * 100, System::TOTAL_TRAVEL_MM);

            Serial.printf("Motor Freq: %.1f Hz, Speed: %.3f mm/s, Limit Switch: %s\n", motorFrequency,
                          (motorFrequency / (System::STEPS_PER_REV * System::MICROSTEPS)) * System::LEAD_SCREW_PITCH,

                          limitSwitch != nullptr ? limitSwitch->isTriggered() ? "TRIGGERED" : "clear" : "NONE");

            Serial.println(F("-------------------------------"));
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

    LimitSwitch* MotionController::getLimitSwitch() const
    {
        return limitSwitch;
    }

}  // namespace MotionSystem
