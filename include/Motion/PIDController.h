#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "Config.h"
#include "Encoders/EncoderInterface.h"

namespace MotionSystem
{
    class PIDController
    {
    public:
        PIDController(EncoderInterface* encoder);
        ~PIDController();
        void                   init();
        void                   setTargetPosition(Types::EncoderPosition targetPosition);
        Types::EncoderPosition getTargetPosition() const;
        int32_t                update();
        static void            pidTask(void* parameter);
        void                   startTask();

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