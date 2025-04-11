#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include "Helper/Logger.h"

namespace MotionSystem
{
    class LimitSwitch
    {
    public:
        LimitSwitch();
        ~LimitSwitch();
        void                  init();
        bool                  isTriggered() const;
        void                  reset();
        void                  setEmergencyStop(bool stop);
        bool                  isEmergencyStop() const;
        static void IRAM_ATTR limitSwitchISR(void* arg);

    private:
        volatile bool triggered;
        volatile bool emergencyStop;
    };
}  // namespace MotionSystem

#endif  // LIMIT_SWITCH_H