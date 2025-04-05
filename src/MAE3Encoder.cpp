#include "MAE3Encoder.h"

// Initialize static member
MAE3Encoder* MAE3Encoder::instance = nullptr;

// Floating-point map function
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

MAE3Encoder::MAE3Encoder(uint8_t signalPin)
    : signalPin(signalPin),
      lastPulseWidth(0),
      lastPosition(0.0f),
      lastUpdateTime(0),
      pulseStartTime(0),
      currentPulseWidth(0),
      newPulseAvailable(false)
{
    // Set the static instance pointer
    instance = this;
}

void MAE3Encoder::begin()
{
    pinMode(signalPin, INPUT);

    // Attach interrupt for both rising and falling edges
    attachInterrupt(digitalPinToInterrupt(signalPin), handleInterrupt, CHANGE);

    lastUpdateTime = millis();
}

// Static interrupt handler
void MAE3Encoder::handleInterrupt()
{
    if (instance)
    {
        instance->measurePulse();
    }
}

// Measure pulse width from interrupt
void MAE3Encoder::measurePulse()
{
    unsigned long currentTime = micros();

    if (digitalRead(signalPin) == HIGH)
    {
        // Rising edge - start of pulse
        pulseStartTime = currentTime;
    }
    else
    {
        // Falling edge - end of pulse
        if (pulseStartTime != 0)
        {
            unsigned long pulseWidth = currentTime - pulseStartTime;

            // Filter out invalid pulse widths
            // Valid range: 5-3935 microseconds
            if (pulseWidth > 5 && pulseWidth < 3935)
            {
                currentPulseWidth = pulseWidth;
                newPulseAvailable = true;
            }
        }
    }
}

bool MAE3Encoder::update()
{
    if (!newPulseAvailable)
    {
        return false;
    }

    // Convert pulse width to degrees using floating-point map
    // 0 μs = 0°, 3933 μs = 360°
    float newPosition = mapf(currentPulseWidth, 0, 3933, 0, 360);

    // Ensure position stays within 0-360 range
    if (newPosition < 0.0f)
        newPosition = 0.0f;
    if (newPosition > 360.0f)
        newPosition = 360.0f;

    // Calculate the absolute difference in position
    float positionDiff = fabs(newPosition - lastPosition);

    // Handle wrap-around at 0/360 degrees
    if (positionDiff > 180.0f)
    {
        positionDiff = 360.0f - positionDiff;
    }

    // Only update if the position has changed by more than 0.5 degrees
    if (positionDiff >= 0.5f)
    {
        lastPulseWidth    = currentPulseWidth;
        lastPosition      = newPosition;
        newPulseAvailable = false;
        return true;
    }

    newPulseAvailable = false;
    return false;
}

float MAE3Encoder::getPositionDegrees() const
{
    return lastPosition;
}

uint32_t MAE3Encoder::getPulseWidth() const
{
    return lastPulseWidth;
}