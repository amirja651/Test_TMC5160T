#ifndef MOTION_CONTROLLER_H
#define MOTION_CONTROLLER_H

#include "Encoders/EncoderInterface.h"
#include "Helper/Types.h"
#include "Motion/LimitSwitch.h"
#include "Motors/TmcController.h"
#include "PIDController.h"

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
    namespace Motion
    {
        constexpr uint16_t MAX_SPEED          = 5000;   // Maximum step frequency in Hz
        constexpr uint16_t ACCELERATION       = 10000;  // Steps per second per second
        constexpr uint16_t MOTION_UPDATE_FREQ = 1000;   // PID update frequency in Hz
    }  // namespace Motion

    class MotionController
    {
    public:
        MotionController(EncoderInterface* encoder, TmcController* motor, PIDController* pidController,
                         LimitSwitch* limitSwitch = nullptr);
        ~MotionController();
        void                   begin();
        void                   moveToPosition(Types::MicronPosition positionMicrons);
        void                   moveRelative(Types::MicronPosition distanceMicrons);
        void                   resetRelativeZero();
        bool                   waitForMotionComplete(float toleranceMicrons, uint32_t timeoutMs);
        static void            motionTask(void* parameter);
        void                   startTask();
        void                   setCurrentSpeed(Types::Speed speed);
        Types::Speed           getCurrentSpeed() const;
        void                   printStatusUpdate(bool showStatus = false);
        void                   setAbsoluteZeroPosition(Types::EncoderPosition position);
        void                   setRelativeZeroPosition(Types::EncoderPosition position);
        Types::MicronPosition  getAbsolutePosition();
        Types::MicronPosition  getRelativePosition();
        Types::EncoderPosition getRelativeZeroPosition() const;
        LimitSwitch*           getLimitSwitch() const;

    private:
        EncoderInterface*      encoder;
        TmcController*         motor;
        PIDController*         pidController;
        LimitSwitch*           limitSwitch;
        Types::Speed           currentSpeed;
        uint64_t               lastStepTime;
        TaskHandle_t           taskHandle;
        char                   buffer[128];
        Types::EncoderPosition absoluteZeroPosition;
        Types::EncoderPosition relativeZeroPosition;
    };
}  // namespace MotionSystem

#endif  // MOTION_CONTROLLER_H