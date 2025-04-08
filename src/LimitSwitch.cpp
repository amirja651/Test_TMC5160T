#include "LimitSwitch.h"

namespace MotionSystem
{

    // Static instance pointer for ISR access
    static LimitSwitch* limitSwitchInstance = nullptr;

    LimitSwitch::LimitSwitch() : triggered(false), emergencyStop(false)
    {
        limitSwitchInstance = this;
    }

    LimitSwitch::~LimitSwitch()
    {
        // Detach interrupt
        detachInterrupt(digitalPinToInterrupt(Config::Pins::LIMIT_SWITCH_PIN));
        // Remove static instance pointer
        if (limitSwitchInstance == this)
        {
            limitSwitchInstance = nullptr;
        }
    }

    void LimitSwitch::init()
    {
        // Configure pin with internal pull-up resistor
        pinMode(Config::Pins::LIMIT_SWITCH_PIN, INPUT_PULLUP);

        // Attach interrupt for limit switch (trigger on falling edge - switch closed)
        attachInterrupt(
            digitalPinToInterrupt(Config::Pins::LIMIT_SWITCH_PIN), []() { LimitSwitch::limitSwitchISR(nullptr); },
            FALLING);

        Serial.print(F("Limit switch configured on pin "));
        Serial.println(String(Config::Pins::LIMIT_SWITCH_PIN));
    }

    bool LimitSwitch::isTriggered() const
    {
        return triggered;
    }

    void LimitSwitch::reset()
    {
        triggered = false;
    }

    void LimitSwitch::setEmergencyStop(bool stop)
    {
        emergencyStop = stop;
    }

    bool LimitSwitch::isEmergencyStop() const
    {
        return emergencyStop;
    }

    void IRAM_ATTR LimitSwitch::limitSwitchISR(void* arg)
    {
        if (limitSwitchInstance)
        {
            limitSwitchInstance->triggered     = true;
            limitSwitchInstance->emergencyStop = true;
        }
    }

}  // namespace MotionSystem

// End of code