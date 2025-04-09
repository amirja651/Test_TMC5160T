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
        lastPidTime         = esp_timer_get_time();
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
        Types::EncoderPosition currentPosition = encoder->readPosition();
        float                  error           = targetPosition - currentPosition;
        uint64_t               now             = esp_timer_get_time();
        float                  dt              = (now - lastPidTime) / 1000000.0;  // Convert to seconds
        lastPidTime                            = now;
        float proportional                     = Config::PID::KP * error;
        integral += Config::PID::KI * error * dt;
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

        output              = proportional + integral + derivative;
        lastError           = error;
        lastEncoderPosition = currentPosition;
        return output;
    }

    void PIDController::pidTask(void* parameter)
    {
        PIDController* controller      = static_cast<PIDController*>(parameter);
        controller->lastPidTime        = esp_timer_get_time();
        TickType_t       xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency    = 1000 / Config::Motion::PID_UPDATE_FREQ;  // Convert to milliseconds
        while (true)
        {
            controller->update();
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
