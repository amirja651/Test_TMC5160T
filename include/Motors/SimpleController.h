#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "Config.h"
#include "Helper/Logger.h"

namespace MotionSystem
{
    class SimpleController
    {
    public:
        SimpleController();
        ~SimpleController() = default;
        void                begin();
        void                enableDriver(bool enable);
        void                setDirection(bool forward);
        void IRAM_ATTR      step();
        uint32_t            calculateStepInterval(Types::Speed speed);
        Types::StepPosition micronsToSteps(Types::MicronPosition microns);
        Types::StepPosition pixelsToSteps(Types::PixelPosition pixels);

    private:
        Types::StepPosition currentStepPosition;
    };
}  // namespace MotionSystem

#endif  // STEPPER_MOTOR_H