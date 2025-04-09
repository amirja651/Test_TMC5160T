#ifndef STATUS_REPORTER_H
#define STATUS_REPORTER_H

#include "Config.h"
#include "Encoders\EncoderInterface.h"
#include "LimitSwitch.h"
#include "PIDController.h"
#include "Types.h"

namespace MotionSystem
{
    class StatusReporter
    {
    public:
        StatusReporter(EncoderInterface* encoder, PIDController* pidController, LimitSwitch* limitSwitch);
        ~StatusReporter();
        void                   printStatusUpdate(bool showStatus = false);
        void                   setCurrentSpeed(Types::Speed speed);
        void                   setAbsoluteZeroPosition(Types::EncoderPosition position);
        void                   setRelativeZeroPosition(Types::EncoderPosition position);
        Types::MicronPosition  getAbsolutePosition();
        Types::MicronPosition  getRelativePosition();
        Types::EncoderPosition getRelativeZeroPosition() const;
        static void            statusTask(void* parameter);
        void                   startTask();

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