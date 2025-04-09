#pragma once

#include "DifferentialEncoder.h"
#include "EncoderConfig.h"
#include "EncoderInterface.h"
#include "PWMEncoder.h"

namespace MotionSystem
{
    class EncoderFactory
    {
    public:
        enum class EncoderType
        {
            DIFFERENTIAL,
            PWM
        };

        static EncoderInterface* createEncoder(EncoderType type, const EncoderConfig& config)
        {
            switch (type)
            {
                case EncoderType::DIFFERENTIAL:
                    return new DifferentialEncoder(config);
                case EncoderType::PWM:
                    return new PWMEncoder(config);
                default:
                    return nullptr;
            }
        }

        // Helper methods for common configurations
        static EncoderConfig createDifferentialConfig(uint8_t pinA, uint8_t pinB, uint8_t pcntUnit = 0)
        {
            EncoderConfig config;
            config.pinA                = pinA;
            config.pinB                = pinB;
            config.pcntUnit            = pcntUnit;
            config.countsPerRevolution = 4000.0f;  // Default for quadrature encoders
            return config;
        }

        static EncoderConfig createPWMConfig(uint8_t signalPin, uint8_t interruptPin)
        {
            EncoderConfig config;
            config.signalPin           = signalPin;
            config.interruptPin        = interruptPin;
            config.countsPerRevolution = 4096.0f;  // 12-bit resolution
            return config;
        }
    };
}  // namespace MotionSystem