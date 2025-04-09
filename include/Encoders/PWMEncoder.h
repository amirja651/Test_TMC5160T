#ifndef PWM_ENCODER_H
#define PWM_ENCODER_H

#include "Config.h"
#include "EncoderConfig.h"
#include "EncoderInterface.h"
#include "EncoderInterruptManager.h"
#include "Types.h"

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
        Types::MicronPosition  countsToMicrons(Types::EncoderPosition counts) override;
        Types::EncoderPosition micronsToEncCounts(Types::MicronPosition microns) override;
        Types::PixelPosition   countsToPixels(Types::EncoderPosition counts) override;

    private:
        static void IRAM_ATTR handleInterrupt(void* arg);
        void                  measurePulse();

        EncoderConfig                   config;
        volatile Types::EncoderPosition position;
        volatile int32_t                overflowCount;
        volatile unsigned long          pulseStartTime;
        volatile unsigned long          currentPulseWidth;
        volatile bool                   newPulseAvailable;
        unsigned long                   lastUpdateTime;

        static constexpr uint32_t MIN_PULSE_WIDTH   = 5;
        static constexpr uint32_t MAX_PULSE_WIDTH   = 3935;
        static constexpr float    DEGREES_PER_PULSE = 360.0f / 4096.0f;  // 12-bit resolution
    };
}  // namespace MotionSystem

#endif  // PWM_ENCODER_H