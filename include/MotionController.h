#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "Config.h"
#include "Encoders\EncoderInterface.h"
#include "LimitSwitch.h"
#include "MotorControllers\SimpleController.h"
#include "PIDController.h"
#include "StatusReporter.h"
#include "Types.h"

const char errorMessage[] PROGMEM   = "ERROR: Position %.3f µm exceeds relative travel limits (±%.1f mm)";
const char errorMessage2[] PROGMEM  = "ERROR: Position %.3f px (%.3f µm) exceeds relative travel limits (±%.1f mm)";
const char errorMessage3[] PROGMEM  = "ERROR: Relative move to %.3f µm exceeds relative travel limits (±%.1f mm)";
const char errorMessage4[] PROGMEM  = "ERROR: Relative move to %.3f µm exceeds relative travel limits (±%.1f mm)";
const char movingMessage[] PROGMEM  = "Moving to: %.3f µm (relative) (Target encoder: %ld)";
const char movingMessage2[] PROGMEM = "Moving to: %.3f px (%.3f µm relative) (Target encoder: %d)";
const char movingMessage3[] PROGMEM = "Moving: %.3f µm (%.3f px) (Target encoder: %d)";
const char movingMessage4[] PROGMEM = "Moving: %.3f px (%.3f µm) (Target encoder: %d)";
namespace MotionSystem
{
    class MotionController
    {
    public:
        MotionController(EncoderInterface* encoder, SimpleController* motor, PIDController* pidController,
                         LimitSwitch* limitSwitch, StatusReporter* statusReporter);
        ~MotionController();
        void         init();
        void         moveToPosition(Types::MicronPosition positionMicrons, bool calibration = false);
        void         moveToPositionPixels(Types::PixelPosition positionPixels);
        void         moveRelative(Types::MicronPosition distanceMicrons);
        void         moveRelativePixels(Types::PixelPosition distancePixels);
        void         calibrateSystem();
        void         resetRelativeZero();
        bool         waitForMotionComplete(float toleranceMicrons, uint32_t timeoutMs);
        void         processCommands();
        static void  motionTask(void* parameter);
        void         startTask();
        void         setCurrentSpeed(Types::Speed speed);
        Types::Speed getCurrentSpeed() const;

    private:
        EncoderInterface*     encoder;
        SimpleController*     motor;
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