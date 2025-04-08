#ifndef ESP32_ENCODER_H
#define ESP32_ENCODER_H

#include <driver/pcnt.h>
#include "Config.h"
#include "EncoderInterface.h"

namespace MotionSystem {

    /**
     * ESP32-specific encoder implementation using hardware pulse counter
     */
    class ESP32Encoder : public EncoderInterface {
    public:
        ESP32Encoder();
        ~ESP32Encoder() override;

        /**
         * Initialize ESP32 pulse counter hardware
         */
        void init() override;

        /**
         * Read the current encoder position
         * @return Current position in encoder counts
         */
        Types::EncoderPosition readPosition() override;

        /**
         * Reset the encoder position to zero
         */
        void resetPosition() override;

        /**
         * Convert from encoder counts to microns
         * @param counts Encoder position in counts
         * @return Position in microns
         */
        Types::MicronPosition countsToMicrons(Types::EncoderPosition counts) override;

        /**
         * Convert from microns to encoder counts
         * @param microns Position in microns
         * @return Position in encoder counts
         */
        Types::EncoderPosition micronsToEncCounts(Types::MicronPosition microns) override;

        /**
         * Convert from encoder counts to pixels
         * @param counts Encoder position in counts
         * @return Position in pixels
         */
        Types::PixelPosition countsToPixels(Types::EncoderPosition counts) override;

        /**
         * Static ISR handler for pulse counter overflow
         * @param arg User argument passed to the ISR
         */
        static void IRAM_ATTR encoderOverflowISR(void* arg);

    private:
        pcnt_unit_t                     encoderPcntUnit;  // ESP32 pulse counter unit
        volatile Types::EncoderPosition position;         // Extended position counter
    };

}  // namespace MotionSystem

#endif  // ESP32_ENCODER_H