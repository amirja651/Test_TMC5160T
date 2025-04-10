#ifndef ENCODER_INTERFACE_H
#define ENCODER_INTERFACE_H

#include "Types.h"

namespace MotionSystem
{
    class EncoderInterface
    {
    public:
        virtual ~EncoderInterface()                    = default;
        virtual void                   begin()         = 0;
        virtual void                   resetPosition() = 0;
        virtual Types::EncoderPosition readPosition()  = 0;
    };
}  // namespace MotionSystem

#endif  // ENCODER_INTERFACE_H