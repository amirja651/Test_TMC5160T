#include "StatusReporter.h"

namespace MotionSystem
{
    StatusReporter::StatusReporter(EncoderInterface* encoder, PIDController* pidController, LimitSwitch* limitSwitch)
        : encoder(encoder),
          pidController(pidController),
          limitSwitch(limitSwitch),
          absoluteZeroPosition(0),
          relativeZeroPosition(0),
          currentSpeed(0),
          taskHandle(nullptr)
    {
    }

    StatusReporter::~StatusReporter()
    {
        if (taskHandle != nullptr)
        {
            vTaskDelete(taskHandle);
            taskHandle = nullptr;
        }
    }

    void StatusReporter::printStatusUpdate(bool showStatus)
    {
        Types::MicronPosition relPosition = getRelativePosition();
        Types::MicronPosition absPosition = getAbsolutePosition();
        Types::MicronPosition relTarget =
            encoder->countsToMicrons(pidController->getTargetPosition() - relativeZeroPosition);
        float error            = relTarget - relPosition;
        float motorFrequency   = abs(currentSpeed);  // Steps per second
        float relTravelPercent = (relPosition / Config::System::REL_TRAVEL_LIMIT_MICRONS) * 100;
        if (motorFrequency > 10 || showStatus)
        {
            Serial.printf("POS(rel): %.3f µm (%.1f%% of ±%.1f mm), TARGET: %.3f µm, ERROR: %.3f µm\n", relPosition,
                          abs(relTravelPercent), Config::System::REL_TRAVEL_LIMIT_MM, relTarget, error);
            Serial.printf("POS(abs): %.3f µm, Travel: %.1f%% of %.1f mm\n", absPosition,
                          (absPosition / Config::System::TOTAL_TRAVEL_MICRONS) * 100, Config::System::TOTAL_TRAVEL_MM);
            Serial.printf("Motor Freq: %.1f Hz, Speed: %.3f mm/s, Limit Switch: %s\n", motorFrequency,
                          (motorFrequency / (Config::System::STEPS_PER_REV * Config::System::MICROSTEPS)) *
                              Config::System::LEAD_SCREW_PITCH,
                          limitSwitch->isTriggered() ? "TRIGGERED" : "clear");
            Serial.println(F("-------------------------------"));
        }
    }

    void StatusReporter::setCurrentSpeed(Types::Speed speed)
    {
        currentSpeed = speed;
    }

    void StatusReporter::setAbsoluteZeroPosition(Types::EncoderPosition position)
    {
        absoluteZeroPosition = position;
    }

    void StatusReporter::setRelativeZeroPosition(Types::EncoderPosition position)
    {
        relativeZeroPosition = position;
    }

    Types::MicronPosition StatusReporter::getAbsolutePosition()
    {
        Types::EncoderPosition currentPosition = encoder->readPosition();
        return encoder->countsToMicrons(currentPosition - absoluteZeroPosition);
    }

    Types::MicronPosition StatusReporter::getRelativePosition()
    {
        Types::EncoderPosition currentPosition = encoder->readPosition();
        return encoder->countsToMicrons(currentPosition - relativeZeroPosition);
    }

    Types::EncoderPosition StatusReporter::getRelativeZeroPosition() const
    {
        return relativeZeroPosition;
    }

    void StatusReporter::statusTask(void* parameter)
    {
        StatusReporter*  reporter      = static_cast<StatusReporter*>(parameter);
        TickType_t       xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency    = Config::System::STATUS_UPDATE_MS;
        while (true)
        {
            Types::MicronPosition relPosition = reporter->getRelativePosition();
            Types::MicronPosition relTarget   = reporter->encoder->countsToMicrons(
                reporter->pidController->getTargetPosition() - reporter->relativeZeroPosition);
            if (abs(relPosition - relTarget) > 0.2)
            {
                reporter->printStatusUpdate();
            }

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    void StatusReporter::startTask()
    {
        xTaskCreatePinnedToCore(statusTask, "Status Updates", Config::Tasks::STATUS_TASK_STACK_SIZE, this,
                                Config::Tasks::STATUS_TASK_PRIORITY, &taskHandle, Config::Tasks::STATUS_TASK_CORE);
        Serial.println(F("Status reporting task started"));
    }

}  // namespace MotionSystem
