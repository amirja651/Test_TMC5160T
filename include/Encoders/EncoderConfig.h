#pragma once

#include <Arduino.h>

namespace MotionSystem
{
    struct EncoderConfig
    {
        uint8_t pinA;          // For differential encoder
        uint8_t pinB;          // For differential encoder
        uint8_t signalPin;     // For PWM encoder
        uint8_t interruptPin;  // For PWM encoder
        uint8_t pcntUnit;      // For differential encoder
        uint8_t priority;      // Interrupt priority
        bool    invertDirection;
        float   countsPerRevolution;
        float   micronsPerCount;

        // Default constructor with reasonable defaults
        EncoderConfig()
            : pinA(0),
              pinB(0),
              signalPin(0),
              interruptPin(0),
              pcntUnit(0),
              priority(1),
              invertDirection(false),
              countsPerRevolution(4000.0f)  // Default for quadrature encoders
              ,
              micronsPerCount(0.5f)  // Default value, should be calibrated
        {
        }
    };
}  // namespace MotionSystem