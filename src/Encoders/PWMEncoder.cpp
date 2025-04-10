#include "Encoders/PWMEncoder.h"
#include "Encoders/EncoderInterruptManager.h"

namespace MotionSystem
{
    PWMEncoder::PWMEncoder(const EncoderConfig& config)
        : config(config),
          position(0),
          pulseStartTime(0),
          currentPulseWidth(0),
          newPulseAvailable(false),
          lastUpdateTime(0)
    {
    }

    PWMEncoder::~PWMEncoder()
    {
        EncoderInterruptManager::getInstance().freeInterrupt(config.interruptPin);
    }

    void PWMEncoder::begin()
    {
        pinMode(config.signalPin, INPUT);

        if (!EncoderInterruptManager::getInstance().allocateInterrupt(config.interruptPin, handleInterrupt, this))
        {
            // Handle error - interrupt allocation failed
            return;
        }

        resetPosition();
    }

    void PWMEncoder::resetPosition()
    {
        position      = 0;
        overflowCount = 0;
    }

    Types::EncoderPosition PWMEncoder::readPosition()
    {
        if (newPulseAvailable)
        {
            measurePulse();
        }
        return position;
    }

    void IRAM_ATTR PWMEncoder::handleInterrupt(void* arg)
    {
        PWMEncoder*   encoder     = static_cast<PWMEncoder*>(arg);
        unsigned long currentTime = micros();

        if (digitalRead(encoder->config.signalPin) == HIGH)
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

            if (config.invertDirection)
            {
                degrees = 360.0f - degrees;
            }

            position = static_cast<Types::EncoderPosition>((degrees / 360.0f) * config.countsPerRevolution);
        }

        newPulseAvailable = false;
        lastUpdateTime    = millis();
    }
}  // namespace MotionSystem