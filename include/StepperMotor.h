#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "Config.h"
#include "Types.h"

namespace MotionSystem {

    /**
     * Controls a stepper motor via step/dir interface
     */
    class StepperMotor {
    public:
        StepperMotor();
        ~StepperMotor() = default;

        /**
         * Initialize the motor driver pins
         */
        void init();

        /**
         * Enable the motor driver
         * @param enable True to enable, false to disable
         */
        void enable(bool enable);

        /**
         * Set motor direction
         * @param dir Direction (true = forward, false = backward)
         */
        void setDirection(bool dir);

        /**
         * Generate a single step pulse
         * Properly timed for stepper driver requirements
         */
        void IRAM_ATTR generateStep();

        /**
         * Calculate the time interval between steps based on speed
         * @param speed Step frequency in Hz
         * @return Time interval in microseconds
         */
        uint32_t calculateStepInterval(Types::Speed speed);

        /**
         * Convert from microns to motor steps
         * @param microns Position in microns
         * @return Position in motor steps
         */
        Types::StepPosition micronsToSteps(Types::MicronPosition microns);

        /**
         * Convert from pixels to motor steps
         * @param pixels Position in pixels
         * @return Position in motor steps
         */
        Types::StepPosition pixelsToSteps(Types::PixelPosition pixels);

    private:
        Types::StepPosition currentStepPosition;
    };

}  // namespace MotionSystem

#endif  // STEPPER_MOTOR_H