#ifndef PWM_ENCODER_H
#define PWM_ENCODER_H

#include <atomic>
#include "EncoderConfig.h"
#include "EncoderInterface.h"
#include "Helper/Types.h"

namespace MotionSystem
{
    class PWMEncoder : public EncoderInterface
    {
    public:
        PWMEncoder(const EncoderConfig& config);
        ~PWMEncoder() override;

        void                   begin() override;
        void                   resetPosition() override;
        Types::EncoderPosition readPosition() override;

    private:
        static void IRAM_ATTR handleInterrupt(void* arg);
        void                  measurePulse();
        uint8_t               readPinFast() const;

        EncoderConfig                       encoderConfig;
        std::atomic<Types::EncoderPosition> position;
        std::atomic<int32_t>                overflowCount;
        volatile unsigned long              pulseStartTime;
        volatile unsigned long              currentPulseWidth;
        volatile bool                       newPulseAvailable;
        unsigned long                       lastUpdateTime;
        uint32_t                            gpioMask;  // For fast pin reading

        static constexpr uint32_t MIN_PULSE_WIDTH   = 5;
        static constexpr uint32_t MAX_PULSE_WIDTH   = 3935;
        static constexpr float    DEGREES_PER_PULSE = 360.0f / 4096.0f;  // 12-bit resolution
    };
}  // namespace MotionSystem

#endif  // PWM_ENCODER_H