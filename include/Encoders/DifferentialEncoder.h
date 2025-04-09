#ifndef ESP32_ENCODER_H
#define ESP32_ENCODER_H

#include <driver/pcnt.h>
#include "Config.h"
#include "EncoderInterface.h"

namespace MotionSystem
{
    class DifferentialEncoder : public EncoderInterface
    {
    public:
        DifferentialEncoder();
        ~DifferentialEncoder() override;
        void                   begin() override;
        void                   resetPosition() override;
        Types::EncoderPosition readPosition() override;
        Types::MicronPosition  countsToMicrons(Types::EncoderPosition counts) override;
        Types::EncoderPosition micronsToEncCounts(Types::MicronPosition microns) override;
        Types::PixelPosition   countsToPixels(Types::EncoderPosition counts) override;
        static void IRAM_ATTR  encoderOverflowISR(void* arg);

    private:
        pcnt_unit_t                     encoderPcntUnit;
        volatile Types::EncoderPosition position;
    };
}  // namespace MotionSystem

#endif  // ESP32_ENCODER_H