#include "Encoders/PWMEncoder.h"

namespace MotionSystem
{
    PWMEncoder::PWMEncoder(const EncoderConfig& config)
        : encoderConfig(config),
          position(0),
          overflowCount(0),
          pulseStartTime(0),
          currentPulseWidth(0),
          newPulseAvailable(false),
          lastUpdateTime(0),
          gpioMask(1ULL << config.signalPin)
    {
    }

    PWMEncoder::~PWMEncoder()
    {
        EncoderInterruptManager::getInstance().freeInterrupt(encoderConfig.interruptPin);
    }

    void PWMEncoder::begin()
    {
        pinMode(encoderConfig.signalPin, INPUT);

        if (!EncoderInterruptManager::getInstance().allocateInterrupt(encoderConfig.interruptPin, handleInterrupt,
                                                                      this))
        {
            // Handle error - interrupt allocation failed
            return;
        }

        resetPosition();
    }

    void PWMEncoder::resetPosition()
    {
        position.store(0);
        overflowCount.store(0);
    }

    Types::EncoderPosition PWMEncoder::readPosition()
    {
        if (newPulseAvailable)
        {
            measurePulse();
        }
        return position.load();
    }

    uint8_t PWMEncoder::readPinFast() const
    {
        return (GPIO.in & gpioMask) ? 1 : 0;
    }

    void IRAM_ATTR PWMEncoder::handleInterrupt(void* arg)
    {
        PWMEncoder*   encoder     = static_cast<PWMEncoder*>(arg);
        unsigned long currentTime = micros();

        if (encoder->readPinFast())
        {
            encoder->pulseStartTime = currentTime;
        }
        else
        {
            encoder->currentPulseWidth = currentTime - encoder->pulseStartTime;
            encoder->newPulseAvailable = true;
        }
    }

    void PWMEncoder::measurePulse()
    {
        if (currentPulseWidth >= MIN_PULSE_WIDTH && currentPulseWidth <= MAX_PULSE_WIDTH)
        {
            float degrees = (currentPulseWidth - MIN_PULSE_WIDTH) * (360.0f / (MAX_PULSE_WIDTH - MIN_PULSE_WIDTH));

            if (encoderConfig.invertDirection)
            {
                degrees = 360.0f - degrees;
            }

            position.store(static_cast<Types::EncoderPosition>((degrees / 360.0f) * encoderConfig.countsPerRevolution));
        }

        newPulseAvailable = false;
        lastUpdateTime    = millis();
    }
}  // namespace MotionSystem