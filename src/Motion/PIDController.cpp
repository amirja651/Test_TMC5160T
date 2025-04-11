#include "Motion/PIDController.h"
#include "Helper/Tasks.h"
#include "Helper/Types.h"

namespace MotionSystem
{
    PIDController::PIDController(const char* name, EncoderInterface* encoder)
        : encoder(encoder),
          instanceName(name),
          targetPosition(0),
          lastEncoderPosition(0),
          integral(0),
          lastError(0),
          output(0),
          lastPidTime(0),
          taskHandle(nullptr),
          kp(PID::KP),
          ki(PID::KI),
          kd(PID::KD),
          maxIntegral(PID::MAX_INTEGRAL),
          maxOutput(1000.0f),  // Default output limit
          shouldStop(false),
          updateCount(0),
          totalUpdateTime(0),
          errorCount(0),
          maxError(0),
          currentError(0),
          currentOutput(0)
    {
    }

    PIDController::~PIDController()
    {
        stopTask();
        if (taskHandle != nullptr)
        {
            vTaskDelete(taskHandle);
            taskHandle = nullptr;
        }
    }

    bool PIDController::validateConfig() const
    {
        if (kp < 0 || ki < 0 || kd < 0)
        {
            Serial.print(instanceName);
            Serial.println(F(" - ERROR: Invalid PID gains"));
            return false;
        }

        if (maxIntegral <= 0)
        {
            Serial.print(instanceName);
            Serial.println(F(" - ERROR: Invalid integral limit"));
            return false;
        }

        if (maxOutput <= 0)
        {
            Serial.print(instanceName);
            Serial.println(F(" - ERROR: Invalid output limit"));
            return false;
        }

        return true;
    }

    void PIDController::init()
    {
        if (!encoder)
        {
            Serial.print(instanceName);
            Serial.println(F(" - ERROR: Encoder not initialized"));
            return;
        }

        if (!validateConfig())
        {
            Serial.print(instanceName);
            Serial.println(F(" - ERROR: Invalid PID configuration"));
            return;
        }

        lastPidTime         = esp_timer_get_time();
        lastEncoderPosition = encoder->readPosition();

        // Single log call for better performance
        char logBuffer[128];
        snprintf(logBuffer, sizeof(logBuffer), " - PID controller initialized with parameters KP:%.2f KI:%.2f KD:%.2f",
                 kp, ki, kd);
        Serial.print(instanceName);
        Serial.println(logBuffer);
    }

    void PIDController::setGains(float kp, float ki, float kd)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    void PIDController::setIntegralLimit(float limit)
    {
        maxIntegral = limit;
    }

    void PIDController::setOutputLimit(float limit)
    {
        maxOutput = limit;
    }

    void PIDController::setTargetPosition(Types::EncoderPosition targetPosition)
    {
        this->targetPosition = targetPosition;
    }

    Types::EncoderPosition PIDController::getTargetPosition() const
    {
        return targetPosition;
    }

    Types::EncoderPosition PIDController::update()
    {
        if (!encoder)
        {
            errorCount++;
            return INVALID_POSITION;
        }

        uint64_t startTime = esp_timer_get_time();

        Types::EncoderPosition currentPosition = encoder->readPosition();
        if (currentPosition == INVALID_POSITION)
        {
            errorCount++;
            return INVALID_POSITION;
        }

        currentError = targetPosition - currentPosition;
        maxError     = max(maxError, abs(currentError));

        uint64_t now = esp_timer_get_time();
        float    dt  = (now - lastPidTime) / 1000000.0f;

        // Prevent division by zero and handle very small time steps
        if (dt < MIN_UPDATE_TIME)
        {
            dt = MIN_UPDATE_TIME;
        }

        // Calculate PID terms
        float proportional = kp * currentError;

        // Anti-windup for integral term
        if (abs(currentError) < maxIntegral)
        {
            integral += ki * currentError * dt;
            // Clamp integral term
            integral = constrain(integral, -maxIntegral, maxIntegral);
        }

        float derivative = 0;
        if (dt > 0)
        {
            derivative = kd * (currentError - lastError) / dt;
        }

        // Calculate output with saturation
        currentOutput = proportional + integral + derivative;
        currentOutput = constrain(currentOutput, -maxOutput, maxOutput);

        lastError           = currentError;
        lastPidTime         = now;
        lastEncoderPosition = currentPosition;

        // Update performance metrics
        updateCount++;
        totalUpdateTime += (esp_timer_get_time() - startTime) / 1000000.0f;

        return static_cast<Types::EncoderPosition>(currentOutput);
    }

    void PIDController::pidTask(void* parameter)
    {
        PIDController* controller = static_cast<PIDController*>(parameter);
        if (!controller)
        {
            vTaskDelete(NULL);
            return;
        }

        controller->lastPidTime        = esp_timer_get_time();
        TickType_t       xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency    = 1000 / PID::PID_UPDATE_FREQ;

        while (true)
        {
            if (controller->shouldStop)
            {
                break;
            }

            Types::EncoderPosition result = controller->update();
            if (result == INVALID_POSITION)
            {
                // Handle error case
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }

        vTaskDelete(NULL);
    }

    void PIDController::startTask()
    {
        if (taskHandle != nullptr)
        {
            Serial.println(F("WARNING: PID task already running"));
            return;
        }

        shouldStop = false;
        xTaskCreatePinnedToCore(pidTask, "PID Control", Tasks::PID_TASK_STACK_SIZE, this, Tasks::PID_TASK_PRIORITY,
                                &taskHandle, Tasks::PID_TASK_CORE);

        Serial.print(instanceName);
        Serial.println(F(" - PID control task started"));
    }

    void PIDController::stopTask()
    {
        shouldStop = true;
        if (taskHandle != nullptr)
        {
            vTaskDelay(pdMS_TO_TICKS(100));  // Give task time to stop
            taskHandle = nullptr;
        }
    }

    bool PIDController::isRunning() const
    {
        return taskHandle != nullptr && !shouldStop;
    }

    void PIDController::getStats(PIDStats& stats) const
    {
        stats.updateCount       = updateCount;
        stats.averageUpdateTime = updateCount > 0 ? totalUpdateTime / updateCount : 0;
        stats.errorCount        = errorCount;
        stats.maxError          = maxError;
        stats.currentError      = currentError;
        stats.currentOutput     = currentOutput;
    }

    float PIDController::getCurrentError() const
    {
        return currentError;
    }

    float PIDController::getCurrentOutput() const
    {
        return currentOutput;
    }
}  // namespace MotionSystem
