#include "Motors/SimpleController.h"

#include "Helper/Logger.h"
#include "Helper/Pins.h"
#include "Helper/System.h"

namespace MotionSystem
{
    SimpleController ::SimpleController() : currentStepPosition(0) {}

    void SimpleController ::begin()
    {
        pinMode(Pins::Simple_Driver::STEP_PIN, OUTPUT);
        pinMode(Pins::Simple_Driver::DIR_PIN, OUTPUT);
        pinMode(Pins::Simple_Driver::ENABLE_PIN, OUTPUT);
        digitalWrite(Pins::Simple_Driver::ENABLE_PIN, LOW);  // Enable the motor driver

        Serial.print(F("Stepper motor initialized on pins STEP:"));
        Serial.print(String(Pins::Simple_Driver::STEP_PIN));
        Serial.print(F(" DIR:"));
        Serial.print(String(Pins::Simple_Driver::DIR_PIN));
        Serial.print(F(" EN:"));
        Serial.println(String(Pins::Simple_Driver::ENABLE_PIN));
    }

    void SimpleController ::enableDriver(bool enable)
    {
        digitalWrite(Pins::Simple_Driver::ENABLE_PIN, enable ? LOW : HIGH);
    }

    void SimpleController ::setDirection(bool dir)
    {
        digitalWrite(Pins::Simple_Driver::DIR_PIN, dir);
        delayMicroseconds(5);  // Direction setup time
    }

    void IRAM_ATTR SimpleController ::step()
    {
        digitalWrite(Pins::Simple_Driver::STEP_PIN, HIGH);
        delayMicroseconds(2);  // Minimum pulse width (check driver specs)
        digitalWrite(Pins::Simple_Driver::STEP_PIN, LOW);
    }

    uint32_t SimpleController ::calculateStepInterval(Types::Speed speed)
    {
        if (abs(speed) < 1)
            return 0;                 // Prevent division by zero
        return 1000000 / abs(speed);  // Convert Hz to microseconds
    }

    Types::StepPosition SimpleController ::micronsToSteps(Types::MicronPosition microns)
    {
        return roundf(microns * System::MOTOR_STEPS_PER_MICRON);
    }

    Types::StepPosition SimpleController ::pixelsToSteps(Types::PixelPosition pixels)
    {
        return micronsToSteps(pixels * System::PIXEL_SIZE);
    }

}  // namespace MotionSystem
