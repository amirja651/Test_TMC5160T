#include "Encoders\DifferentialEncoder.h"

namespace MotionSystem
{

    // Static instance pointer for ISR access
    static DifferentialEncoder* encoderInstance = nullptr;

    DifferentialEncoder::DifferentialEncoder() : encoderPcntUnit(PCNT_UNIT_0), position(0)
    {
        encoderInstance = this;
    }

    DifferentialEncoder::~DifferentialEncoder()
    {
        // Clean up ISR handler
        pcnt_isr_handler_remove(encoderPcntUnit);
        // Remove static instance pointer
        if (encoderInstance == this)
        {
            encoderInstance = nullptr;
        }
    }

    void DifferentialEncoder::init()
    {
        // Configure pulse counter unit for quadrature decoding
        pcnt_config_t pcntConfig = {
            .pulse_gpio_num = Config::Pins::ENCODER_A_PIN,  // Channel A
            .ctrl_gpio_num  = Config::Pins::ENCODER_B_PIN,  // Channel B
            .lctrl_mode     = PCNT_MODE_REVERSE,            // Reverse counting when B=1
            .hctrl_mode     = PCNT_MODE_KEEP,               // Keep counting when B=0
            .pos_mode       = PCNT_COUNT_INC,               // Count up on rising edge A
            .neg_mode       = PCNT_COUNT_DEC,               // Count down on falling edge A
            .counter_h_lim  = 32767,                        // Max hardware limit
            .counter_l_lim  = -32768,                       // Min hardware limit
            .unit           = encoderPcntUnit,
            .channel        = PCNT_CHANNEL_0,
        };

        pcnt_unit_config(&pcntConfig);

        // Set up pulse counter events
        pcnt_event_enable(encoderPcntUnit, PCNT_EVT_H_LIM);
        pcnt_event_enable(encoderPcntUnit, PCNT_EVT_L_LIM);

        // Filter out glitches (adjust these values based on encoder specs)
        pcnt_set_filter_value(encoderPcntUnit, 100);
        pcnt_filter_enable(encoderPcntUnit);

        // Start the counter
        pcnt_counter_pause(encoderPcntUnit);
        pcnt_counter_clear(encoderPcntUnit);
        pcnt_counter_resume(encoderPcntUnit);

        // Set up interrupt for overflow handling
        pcnt_isr_service_install(0);
        pcnt_isr_handler_add(encoderPcntUnit, DifferentialEncoder::encoderOverflowISR, nullptr);

        Serial.print(F("ESP32 encoder initialized on pins A:"));
        Serial.print(String(Config::Pins::ENCODER_A_PIN));
        Serial.print(F(" B:"));
        Serial.println(String(Config::Pins::ENCODER_B_PIN));
    }

    Types::EncoderPosition DifferentialEncoder::readPosition()
    {
        int16_t count = 0;
        pcnt_get_counter_value(encoderPcntUnit, &count);
        return position + count;
    }

    void DifferentialEncoder::resetPosition()
    {
        position = 0;
        pcnt_counter_clear(encoderPcntUnit);
    }

    Types::MicronPosition DifferentialEncoder::countsToMicrons(Types::EncoderPosition counts)
    {
        return static_cast<Types::MicronPosition>(counts) / Config::System::ENCODER_COUNTS_PER_MICRON;
    }

    Types::EncoderPosition DifferentialEncoder::micronsToEncCounts(Types::MicronPosition microns)
    {
        return static_cast<Types::EncoderPosition>(microns * MotionSystem::Config::System::ENCODER_COUNTS_PER_MICRON);
    }

    Types::PixelPosition DifferentialEncoder::countsToPixels(Types::EncoderPosition counts)
    {
        return countsToMicrons(counts) / Config::System::PIXEL_SIZE;
    }

    void IRAM_ATTR DifferentialEncoder::encoderOverflowISR(void* arg)
    {
        if (!encoderInstance)
        {
            return;
        }

        uint32_t status = 0;
        pcnt_get_event_status(encoderInstance->encoderPcntUnit, &status);

        if (status & PCNT_EVT_H_LIM)
        {
            encoderInstance->position += 32767;
            pcnt_counter_clear(encoderInstance->encoderPcntUnit);
        }
        else if (status & PCNT_EVT_L_LIM)
        {
            encoderInstance->position -= 32768;
            pcnt_counter_clear(encoderInstance->encoderPcntUnit);
        }
    }

}  // namespace MotionSystem

// End of code