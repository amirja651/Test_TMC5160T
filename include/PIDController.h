#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "Config.h"
#include "EncoderInterface.h"
#include "Types.h"

namespace MotionSystem {

    /**
     * PID controller for closed-loop position control
     */
    class PIDController {
    public:
        PIDController(EncoderInterface* encoder);
        ~PIDController();

        /**
         * Initialize PID controller
         */
        void init();

        /**
         * Set the target position in encoder counts
         * @param targetPosition Target position in encoder counts
         */
        void setTargetPosition(Types::EncoderPosition targetPosition);

        /**
         * Get the current target position
         * @return Target position in encoder counts
         */
        Types::EncoderPosition getTargetPosition() const;

        /**
         * Update the PID controller
         * Should be called at the PID update frequency
         * @return Control output
         */
        int32_t update();

        /**
         * Task function for running the PID control loop
         * @param parameter Task parameter (pointer to PIDController)
         */
        static void pidTask(void* parameter);

        /**
         * Start the PID control task
         */
        void startTask();

    private:
        EncoderInterface*      encoder;
        Types::EncoderPosition targetPosition;
        Types::EncoderPosition lastEncoderPosition;
        float                  integral;
        float                  lastError;
        int32_t                output;
        uint64_t               lastPidTime;
        TaskHandle_t           taskHandle;
    };

}  // namespace MotionSystem

#endif  // PID_CONTROLLER_H