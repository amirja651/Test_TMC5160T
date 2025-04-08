#include "PIDController.h"
#include "esp_timer.h"

namespace MotionSystem
{

    PIDController::PIDController(EncoderInterface* encoder)
        : encoder(encoder),
          targetPosition(0),
          lastEncoderPosition(0),
          integral(0),
          lastError(0),
          output(0),
          lastPidTime(0),
          taskHandle(nullptr)
    {
    }

    PIDController::~PIDController()
    {
        if (taskHandle != nullptr)
        {
            vTaskDelete(taskHandle);
            taskHandle = nullptr;
        }
    }

    void PIDController::init()
    {
        // Initialize timing
        lastPidTime = esp_timer_get_time();

        // Initialize starting position
        lastEncoderPosition = encoder->readPosition();

        Serial.println(F("PID controller initialized with parameters KP:"));
        Serial.println(String(Config::PID::KP));
        Serial.println(F("KI:"));
        Serial.println(String(Config::PID::KI));
        Serial.println(F("KD:"));
        Serial.println(String(Config::PID::KD));
    }

    void PIDController::setTargetPosition(Types::EncoderPosition targetPosition)
    {
        this->targetPosition = targetPosition;
    }

    Types::EncoderPosition PIDController::getTargetPosition() const
    {
        return targetPosition;
    }

    int32_t PIDController::update()
    {
        // Get current position
        Types::EncoderPosition currentPosition = encoder->readPosition();

        // Calculate error (in encoder counts)
        float error = targetPosition - currentPosition;

        // Calculate time delta
        uint64_t now = esp_timer_get_time();
        float    dt  = (now - lastPidTime) / 1000000.0;  // Convert to seconds
        lastPidTime  = now;

        // Calculate PID terms
        float proportional = Config::PID::KP * error;
        integral += Config::PID::KI * error * dt;

        // Apply anti-windup
        if (integral > Config::PID::MAX_INTEGRAL)
        {
            integral = Config::PID::MAX_INTEGRAL;
        }
        if (integral < -Config::PID::MAX_INTEGRAL)
        {
            integral = -Config::PID::MAX_INTEGRAL;
        }

        float derivative = 0;
        if (dt > 0)
        {
            derivative = Config::PID::KD * (error - lastError) / dt;
        }

        // Calculate output
        output = proportional + integral + derivative;

        // Store values for next iteration
        lastError           = error;
        lastEncoderPosition = currentPosition;

        return output;
    }

    void PIDController::pidTask(void* parameter)
    {
        PIDController* controller = static_cast<PIDController*>(parameter);

        // Initialize timing
        controller->lastPidTime = esp_timer_get_time();

        TickType_t       xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency    = 1000 / Config::Motion::PID_UPDATE_FREQ;  // Convert to milliseconds

        while (true)
        {
            controller->update();

            // Use vTaskDelayUntil for precise timing
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
    }

    void PIDController::startTask()
    {
        xTaskCreatePinnedToCore(pidTask, "PID Control", Config::Tasks::PID_TASK_STACK_SIZE, this,
                                Config::Tasks::PID_TASK_PRIORITY, &taskHandle, Config::Tasks::PID_TASK_CORE);

        Serial.println(F("PID control task started"));
    }

}  // namespace MotionSystem

// End of code