#include "Motion/LimitSwitch.h"

namespace MotionSystem
{
    static LimitSwitch* limitSwitchInstance = nullptr;
    LimitSwitch::LimitSwitch() : triggered(false), emergencyStop(false)
    {
        limitSwitchInstance = this;
    }

    LimitSwitch::~LimitSwitch()
    {
        detachInterrupt(digitalPinToInterrupt(Config::Pins::LIMIT_SWITCH_PIN));
        if (limitSwitchInstance == this)
        {
            limitSwitchInstance = nullptr;
        }
    }

    void LimitSwitch::init()
    {
        pinMode(Config::Pins::LIMIT_SWITCH_PIN, INPUT_PULLUP);
        attachInterrupt(
            digitalPinToInterrupt(Config::Pins::LIMIT_SWITCH_PIN), []() { LimitSwitch::limitSwitchISR(nullptr); },
            FALLING);
        Logger::getInstance().log(F("Limit switch configured on pin "));
        Logger::getInstance().logln(String(Config::Pins::LIMIT_SWITCH_PIN));
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
