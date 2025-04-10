#include "Encoders/DifferentialEncoder.h"
#include "Encoders/EncoderInterruptManager.h"

namespace MotionSystem
{
    static DifferentialEncoder* encoderInstance = nullptr;
    DifferentialEncoder::DifferentialEncoder(const EncoderConfig& config)
        : config(config), encoderPcntUnit(static_cast<pcnt_unit_t>(config.pcntUnit)), position(0), overflowCount(0)
    {
        encoderInstance = this;
    }

    DifferentialEncoder::~DifferentialEncoder()
    {
        pcnt_counter_pause(encoderPcntUnit);
        pcnt_counter_clear(encoderPcntUnit);
        if (encoderInstance == this)
        {
            encoderInstance = nullptr;
        }
    }

    void DifferentialEncoder::begin()
    {
        setupPCNT();
        resetPosition();
    }

    void DifferentialEncoder::resetPosition()
    {
        position      = 0;
        overflowCount = 0;
        pcnt_counter_clear(encoderPcntUnit);
    }

    Types::EncoderPosition DifferentialEncoder::readPosition()
    {
        int16_t count;
        pcnt_get_counter_value(encoderPcntUnit, &count);

        // Handle overflow/underflow
        if (count >= 32767)
        {
            overflowCount++;
            pcnt_counter_clear(encoderPcntUnit);
        }
        else if (count <= -32768)
        {
            overflowCount--;
            pcnt_counter_clear(encoderPcntUnit);
        }

        position =
            static_cast<Types::EncoderPosition>(count) + (static_cast<Types::EncoderPosition>(overflowCount) * 65536);

        if (config.invertDirection)
        {
            position = -position;
        }

        return position;
    }

    void IRAM_ATTR DifferentialEncoder::encoderOverflowISR(void* arg)
    {
        DifferentialEncoder* encoder = static_cast<DifferentialEncoder*>(arg);
        if (encoder)
        {
            if (pcnt_get_counter_value(encoder->encoderPcntUnit, nullptr) > 0)
            {
                encoder->overflowCount++;
            }
            else
            {
                encoder->overflowCount--;
            }
            pcnt_counter_clear(encoder->encoderPcntUnit);
        }
    }

    void DifferentialEncoder::setupPCNT()
    {
        pcnt_config_t pcnt_config = {
            .pulse_gpio_num = config.pinA,
            .ctrl_gpio_num  = config.pinB,
            .lctrl_mode     = PCNT_MODE_REVERSE,
            .hctrl_mode     = PCNT_MODE_KEEP,
            .pos_mode       = PCNT_COUNT_INC,
            .neg_mode       = PCNT_COUNT_DEC,
            .counter_h_lim  = 32767,
            .counter_l_lim  = -32768,
            .unit           = encoderPcntUnit,
            .channel        = PCNT_CHANNEL_0,
        };

        pcnt_unit_config(&pcnt_config);

        // Configure filter
        pcnt_set_filter_value(encoderPcntUnit, 100);
        pcnt_filter_enable(encoderPcntUnit);

        // Enable events on zero, maximum and minimum limit values
        pcnt_event_enable(encoderPcntUnit, PCNT_EVT_ZERO);
        pcnt_event_enable(encoderPcntUnit, PCNT_EVT_H_LIM);
        pcnt_event_enable(encoderPcntUnit, PCNT_EVT_L_LIM);

        // Register ISR handler
        pcnt_isr_register(encoderOverflowISR, this, 0, nullptr);

        // Enable interrupts
        pcnt_intr_enable(encoderPcntUnit);

        // Start counting
        pcnt_counter_resume(encoderPcntUnit);
    }
}  // namespace MotionSystem
