#ifndef DIFFERENTIAL_ENCODER_H
#define DIFFERENTIAL_ENCODER_H

#include <driver/pcnt.h>
#include "Config.h"
#include "EncoderConfig.h"
#include "EncoderInterface.h"
#include "EncoderInterruptManager.h"

namespace MotionSystem
{
    class DifferentialEncoder : public EncoderInterface
    {
    public:
        DifferentialEncoder(const EncoderConfig& config);
        ~DifferentialEncoder() override;
        void                   begin() override;
        void                   resetPosition() override;
        Types::EncoderPosition readPosition() override;
        static void IRAM_ATTR  encoderOverflowISR(void* arg);

    private:
        void setupPCNT();

        EncoderConfig                   config;
        pcnt_unit_t                     encoderPcntUnit;
        volatile Types::EncoderPosition position;
        volatile int32_t                overflowCount;
    };
}  // namespace MotionSystem

#endif  // DIFFERENTIAL_ENCODER_H