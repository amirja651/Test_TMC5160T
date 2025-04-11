#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "Config.h"
#include "Encoders/EncoderInterface.h"
#include "Helper/Logger.h"
#include "esp_timer.h"

namespace MotionSystem
{
    using namespace MotionSystem::Types;

    class PIDController
    {
    public:
        PIDController(EncoderInterface* encoder);
        ~PIDController();
        void            init();
        void            setTargetPosition(Types::EncoderPosition targetPosition);
        EncoderPosition getTargetPosition() const;
        EncoderPosition update();
        static void     pidTask(void* parameter);
        void            startTask();

    private:
        EncoderInterface* encoder;
        EncoderPosition   targetPosition;
        EncoderPosition   lastEncoderPosition;
        float             integral;
        float             lastError;
        EncoderPosition   output;
        Timestamp         lastPidTime;
        TaskHandle_t      taskHandle;
    };
}  // namespace MotionSystem

#endif  // PID_CONTROLLER_H