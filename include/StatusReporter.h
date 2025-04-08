#ifndef STATUS_REPORTER_H
#define STATUS_REPORTER_H

#include "Config.h"
#include "EncoderInterface.h"
#include "LimitSwitch.h"
#include "PIDController.h"
#include "Types.h"

namespace MotionSystem {

    /**
     * Handles periodic status reporting
     */
    class StatusReporter {
    public:
        StatusReporter(EncoderInterface* encoder, PIDController* pidController,
                       LimitSwitch* limitSwitch);
        ~StatusReporter();

        /**
         * Print a status update to the serial port
         * @param showStatus Force display regardless of motion
         */
        void printStatusUpdate(bool showStatus = false);

        /**
         * Set current speed for status reporting
         * @param speed Current speed in steps/second
         */
        void setCurrentSpeed(Types::Speed speed);

        /**
         * Set absolute zero position
         * @param position Absolute zero in encoder counts
         */
        void setAbsoluteZeroPosition(Types::EncoderPosition position);

        /**
         * Set relative zero position
         * @param position Relative zero in encoder counts
         */
        void setRelativeZeroPosition(Types::EncoderPosition position);

        /**
         * Get absolute position in microns
         * @return Absolute position in microns
         */
        Types::MicronPosition getAbsolutePosition();

        /**
         * Get relative position in microns
         * @return Relative position in microns
         */
        Types::MicronPosition getRelativePosition();

        /**
         * Get the relative zero position
         * @return Relative zero position in encoder counts
         */
        Types::EncoderPosition getRelativeZeroPosition() const;

        /**
         * Task function for periodic status updates
         * @param parameter Task parameter (pointer to StatusReporter)
         */
        static void statusTask(void* parameter);

        /**
         * Start the status update task
         */
        void startTask();

    private:
        EncoderInterface*      encoder;
        PIDController*         pidController;
        LimitSwitch*           limitSwitch;
        Types::EncoderPosition absoluteZeroPosition;
        Types::EncoderPosition relativeZeroPosition;
        Types::Speed           currentSpeed;
        TaskHandle_t           taskHandle;
    };

}  // namespace MotionSystem

#endif  // STATUS_REPORTER_H