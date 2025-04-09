#ifndef ENCODER_INTERFACE_H
#define ENCODER_INTERFACE_H

#include "Types.h"

namespace MotionSystem
{

    /**
     * Abstract interface for position encoders
     */
    class EncoderInterface
    {
    public:
        virtual ~EncoderInterface() = default;

        /**
         * Initialize the encoder hardware
         */
        virtual void init() = 0;

        /**
         * Read the current encoder position
         * @return Current position in encoder counts
         */
        virtual Types::EncoderPosition readPosition() = 0;

        /**
         * Reset the encoder position to zero
         */
        virtual void resetPosition() = 0;

        /**
         * Convert from encoder counts to microns
         * @param counts Encoder position in counts
         * @return Position in microns
         */
        virtual Types::MicronPosition countsToMicrons(Types::EncoderPosition counts) = 0;

        /**
         * Convert from microns to encoder counts
         * @param microns Position in microns
         * @return Position in encoder counts
         */
        virtual Types::EncoderPosition micronsToEncCounts(Types::MicronPosition microns) = 0;

        /**
         * Convert from encoder counts to pixels
         * @param counts Encoder position in counts
         * @return Position in pixels
         */
        virtual Types::PixelPosition countsToPixels(Types::EncoderPosition counts) = 0;
    };

}  // namespace MotionSystem

#endif  // ENCODER_INTERFACE_H