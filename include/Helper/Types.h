#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <functional>

namespace MotionSystem
{  // namespace ESP32Pins
    namespace Types
    {
        using Timestamp        = uint64_t;
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