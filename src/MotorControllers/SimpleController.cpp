#include "MotorControllers\SimpleController.h"

namespace MotionSystem
{
    SimpleController ::SimpleController() : currentStepPosition(0) {}

    void SimpleController ::begin()
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

    void SimpleController ::enableDriver(bool enable)
    {
        digitalWrite(Config::Pins::ENABLE_PIN, enable ? LOW : HIGH);
    }

    void SimpleController ::setDirection(bool dir)
    {
        digitalWrite(Config::Pins::DIR_PIN, dir);
        delayMicroseconds(5);  // Direction setup time
    }

    void IRAM_ATTR SimpleController ::step()
    {
        digitalWrite(Config::Pins::STEP_PIN, HIGH);
        delayMicroseconds(2);  // Minimum pulse width (check driver specs)
        digitalWrite(Config::Pins::STEP_PIN, LOW);
    }

    uint32_t SimpleController ::calculateStepInterval(Types::Speed speed)
    {
        if (abs(speed) < 1)
            return 0;                 // Prevent division by zero
        return 1000000 / abs(speed);  // Convert Hz to microseconds
    }

    Types::StepPosition SimpleController ::micronsToSteps(Types::MicronPosition microns)
    {
        return roundf(microns * Config::System::MOTOR_STEPS_PER_MICRON);
    }

    Types::StepPosition SimpleController ::pixelsToSteps(Types::PixelPosition pixels)
    {
        return micronsToSteps(pixels * Config::System::PIXEL_SIZE);
    }

}  // namespace MotionSystem
