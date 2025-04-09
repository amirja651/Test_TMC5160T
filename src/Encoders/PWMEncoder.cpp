#include "Encoders\PWMEncoder.h"

MotionSystem::PWMEncoder* MotionSystem::PWMEncoder::instance = nullptr;

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

MotionSystem::PWMEncoder::PWMEncoder(uint8_t signalPin)
    : signalPin(signalPin),
      lastPulseWidth(0),
      lastPosition(0.0f),
      lastUpdateTime(0),
      pulseStartTime(0),
      currentPulseWidth(0),
      newPulseAvailable(false)
{
    instance = this;
}

void MotionSystem::PWMEncoder::begin()
{
    pinMode(signalPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(signalPin), handleInterrupt, CHANGE);
    lastUpdateTime = millis();
}

void MotionSystem::PWMEncoder::handleInterrupt()
{
    if (instance)
    {
        if (digitalRead(instance->signalPin) == HIGH)
        {
            instance->pulseStartTime = micros();
        }

        else
        {
            unsigned long currentTime = micros();
            if (instance->pulseStartTime != 0)
            {
                unsigned long pulseWidth = currentTime - instance->pulseStartTime;
                if (pulseWidth > MIN_PULSE_WIDTH && pulseWidth < MAX_PULSE_WIDTH)
                {
                    instance->currentPulseWidth = pulseWidth;
                    instance->newPulseAvailable = true;
                }
            }
        }
    }
}

bool MotionSystem::PWMEncoder::update()
{
    if (!newPulseAvailable)
    {
        return false;
    }

    noInterrupts();
    uint32_t pulseWidth = currentPulseWidth;
    newPulseAvailable   = false;
    interrupts();
    float newPosition = mapf(pulseWidth, 0, 3933, 0, 360);
    if (newPosition < 0.0f)
        newPosition = 0.0f;
    if (newPosition > 360.0f)
        newPosition = 360.0f;
    float positionDiff = fabs(newPosition - lastPosition);
    if (positionDiff > 180.0f)
    {
        positionDiff = 360.0f - positionDiff;
    }

    if (positionDiff >= POSITION_THRESHOLD)
    {
        lastPulseWidth = pulseWidth;
        lastPosition   = newPosition;
        return true;
    }

    return false;
}

float MotionSystem::PWMEncoder::getPositionDegrees() const
{
    return lastPosition;
}

uint32_t MotionSystem::PWMEncoder::getPulseWidth() const
{
    return lastPulseWidth;
}