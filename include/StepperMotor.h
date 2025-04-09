#ifndef STEPPER_MOTOR_H
#define STEPPER_MOTOR_H

#include "Config.h"
#include "Types.h"

namespace MotionSystem
{
    class StepperMotor
    {
    public:
        StepperMotor();
        ~StepperMotor() = default;
        void                init();
        void                enable(bool enable);
        void                setDirection(bool dir);
        void IRAM_ATTR      generateStep();
        uint32_t            calculateStepInterval(Types::Speed speed);
        Types::StepPosition micronsToSteps(Types::MicronPosition microns);
        Types::StepPosition pixelsToSteps(Types::PixelPosition pixels);

    private:
        Types::StepPosition currentStepPosition;
    };
}  // namespace MotionSystem

#endif  // STEPPER_MOTOR_H