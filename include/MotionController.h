#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "Config.h"
#include "EncoderInterface.h"
#include "LimitSwitch.h"
#include "PIDController.h"
#include "StatusReporter.h"
#include "StepperMotor.h"
#include "Types.h"

const char errorMessage[] PROGMEM  = "ERROR: Position %.3f µm exceeds relative travel limits (±%.1f mm)";
const char errorMessage2[] PROGMEM = "ERROR: Position %.3f px (%.3f µm) exceeds relative travel limits (±%.1f mm)";
const char errorMessage3[] PROGMEM = "ERROR: Relative move to %.3f µm exceeds relative travel limits (±%.1f mm)";
const char errorMessage4[] PROGMEM = "ERROR: Relative move to %.3f µm exceeds relative travel limits (±%.1f mm)";

const char movingMessage[] PROGMEM  = "Moving to: %.3f µm (relative) (Target encoder: %ld)";
const char movingMessage2[] PROGMEM = "Moving to: %.3f px (%.3f µm relative) (Target encoder: %d)";
const char movingMessage3[] PROGMEM = "Moving: %.3f µm (%.3f px) (Target encoder: %d)";
const char movingMessage4[] PROGMEM = "Moving: %.3f px (%.3f µm) (Target encoder: %d)";

namespace MotionSystem
{

    /**
     * Main motion control system coordinator
     */
    class MotionController
    {
    public:
        MotionController(EncoderInterface* encoder, StepperMotor* motor, PIDController* pidController,
                         LimitSwitch* limitSwitch, StatusReporter* statusReporter);
        ~MotionController();

        /**
         * Initialize the motion control system
         */
        void init();

        /**
         * Move to absolute position in microns (relative to relative zero)
         * @param positionMicrons Target position in microns
         * @param calibration Whether this is a calibration move
         */
        void moveToPosition(Types::MicronPosition positionMicrons, bool calibration = false);

        /**
         * Move to absolute position in pixels (relative to relative zero)
         * @param positionPixels Target position in pixels
         */
        void moveToPositionPixels(Types::PixelPosition positionPixels);

        /**
         * Move relative to current position in microns
         * @param distanceMicrons Distance to move in microns
         */
        void moveRelative(Types::MicronPosition distanceMicrons);

        /**
         * Move relative to current position in pixels
         * @param distancePixels Distance to move in pixels
         */
        void moveRelativePixels(Types::PixelPosition distancePixels);

        /**
         * Calibrate the system by finding home position
         */
        void calibrateSystem();

        /**
         * Reset the relative zero position to current position
         */
        void resetRelativeZero();

        /**
         * Wait for motion to complete within tolerance
         * @param toleranceMicrons Tolerance in microns
         * @param timeoutMs Timeout in milliseconds
         * @return True if motion completed, false if timeout
         */
        bool waitForMotionComplete(float toleranceMicrons, uint32_t timeoutMs);

        /**
         * Process commands from serial
         */
        void processCommands();

        /**
         * Motion control task function
         * @param parameter Task parameter (pointer to MotionController)
         */
        static void motionTask(void* parameter);

        /**
         * Start the motion control task
         */
        void startTask();

        /**
         * Set the current speed
         * @param speed Speed in steps/second
         */
        void setCurrentSpeed(Types::Speed speed);

        /**
         * Get the current speed
         * @return Current speed in steps/second
         */
        Types::Speed getCurrentSpeed() const;

    private:
        EncoderInterface*     encoder;
        StepperMotor*         motor;
        PIDController*        pidController;
        LimitSwitch*          limitSwitch;
        StatusReporter*       statusReporter;
        Types::Speed          currentSpeed;
        uint64_t              lastStepTime;
        TaskHandle_t          taskHandle;
        Types::MicronPosition pixelsToMicrons(Types::PixelPosition pixels);
        char                  buffer[128];
    };

}  // namespace MotionSystem

#endif  // MOTION_CONTROLLER_H