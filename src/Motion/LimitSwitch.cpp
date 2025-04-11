#include "Motion/LimitSwitch.h"
#include "Helper/Pins.h"

namespace MotionSystem
{
    static LimitSwitch* limitSwitchInstance = nullptr;

    LimitSwitch::LimitSwitch() : triggered(false), emergencyStop(false)
    {
        limitSwitchInstance = this;
    }

    LimitSwitch::~LimitSwitch()
    {
        detachInterrupt(digitalPinToInterrupt(Pins::LimitSwitch::LIMIT_SWITCH_PIN));
        if (limitSwitchInstance == this)
        {
            limitSwitchInstance = nullptr;
        }
    }

    void LimitSwitch::init()
    {
        pinMode(Pins::LimitSwitch::LIMIT_SWITCH_PIN, INPUT_PULLUP);

        attachInterrupt(
            digitalPinToInterrupt(Pins::LimitSwitch::LIMIT_SWITCH_PIN), []() { LimitSwitch::limitSwitchISR(nullptr); },
            FALLING);

        Serial.print(F("Limit switch configured on pin "));
        Serial.println(String(Pins::LimitSwitch::LIMIT_SWITCH_PIN));
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
