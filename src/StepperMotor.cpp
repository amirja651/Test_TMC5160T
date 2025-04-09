#include "StepperMotor.h"

namespace MotionSystem
{
    StepperMotor::StepperMotor() : currentStepPosition(0) {}

    void StepperMotor::init()
    {
        pinMode(Config::Pins::STEP_PIN, OUTPUT);
        pinMode(Config::Pins::DIR_PIN, OUTPUT);
        pinMode(Config::Pins::ENABLE_PIN, OUTPUT);
        digitalWrite(Config::Pins::ENABLE_PIN, LOW);  // Enable the motor driver
        Serial.print(F("Stepper motor initialized on pins STEP:"));
        Serial.print(String(Config::Pins::STEP_PIN));
        Serial.print(F(" DIR:"));
        Serial.print(String(Config::Pins::DIR_PIN));
        Serial.print(F(" EN:"));
        Serial.println(String(Config::Pins::ENABLE_PIN));
    }

    void StepperMotor::enable(bool enable)
    {
        digitalWrite(Config::Pins::ENABLE_PIN, enable ? LOW : HIGH);
    }

    void StepperMotor::setDirection(bool dir)
    {
        digitalWrite(Config::Pins::DIR_PIN, dir);
        delayMicroseconds(5);  // Direction setup time
    }

    void IRAM_ATTR StepperMotor::generateStep()
    {
        digitalWrite(Config::Pins::STEP_PIN, HIGH);
        delayMicroseconds(2);  // Minimum pulse width (check driver specs)
        digitalWrite(Config::Pins::STEP_PIN, LOW);
    }

    uint32_t StepperMotor::calculateStepInterval(Types::Speed speed)
    {
        if (abs(speed) < 1)
            return 0;                 // Prevent division by zero
        return 1000000 / abs(speed);  // Convert Hz to microseconds
    }

    Types::StepPosition StepperMotor::micronsToSteps(Types::MicronPosition microns)
    {
        return roundf(microns * Config::System::MOTOR_STEPS_PER_MICRON);
    }

    Types::StepPosition StepperMotor::pixelsToSteps(Types::PixelPosition pixels)
    {
        return micronsToSteps(pixels * Config::System::PIXEL_SIZE);
    }

}  // namespace MotionSystem
