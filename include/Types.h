#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>
#include <functional>

namespace MotionSystem
{
    namespace Types
    {
        using EncoderPosition  = int32_t;
        using MicronPosition   = float;
        using PixelPosition    = float;
        using StepPosition     = int32_t;
        using Speed            = float;
        using Acceleration     = float;
        using StatusCallback   = std::function<void(bool)>;
        using CompleteCallback = std::function<void(void)>;
        enum class ErrorCode
        {
            SUCCESS = 0,
            LIMIT_SWITCH_TRIGGERED,
            POSITION_OUT_OF_BOUNDS,
            CALIBRATION_FAILED,
            TIMEOUT
        };
    }  // namespace Types
}  // namespace MotionSystem

#endif  // TYPES_H