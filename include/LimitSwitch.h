#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include "Config.h"
#include "Types.h"

namespace MotionSystem {

    /**
     * Handles the limit switch for homing and safety
     */
    class LimitSwitch {
    public:
        LimitSwitch();
        ~LimitSwitch();

        /**
         * Initialize the limit switch with interrupt
         */
        void init();

        /**
         * Check if the limit switch is currently triggered
         * @return True if limit switch is triggered
         */
        bool isTriggered() const;

        /**
         * Reset the limit switch triggered flag
         */
        void reset();

        /**
         * Set the emergency stop flag
         * @param stop True to set emergency stop
         */
        void setEmergencyStop(bool stop);

        /**
         * Check if emergency stop is triggered
         * @return True if emergency stop is active
         */
        bool isEmergencyStop() const;

        /**
         * Static ISR handler for limit switch
         */
        static void IRAM_ATTR limitSwitchISR(void* arg);

    private:
        volatile bool triggered;
        volatile bool emergencyStop;
    };

}  // namespace MotionSystem

#endif  // LIMIT_SWITCH_H