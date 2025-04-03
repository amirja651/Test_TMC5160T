#include "MAE3Encoder.h"

MAE3Encoder::MAE3Encoder(uint8_t pin, bool is12Bit)
    : _pin(pin), _is12Bit(is12Bit), _lastPosition(0) {}

void MAE3Encoder::begin() {
    pinMode(_pin, INPUT);
}

uint16_t MAE3Encoder::readPosition() {
    _lastPosition = readPWM();
    return _lastPosition;
}

float MAE3Encoder::readAngle() {
    return positionToAngle(readPosition());
}

uint16_t MAE3Encoder::readPWM() {
    unsigned long highTime  = pulseIn(_pin, HIGH);
    unsigned long lowTime   = pulseIn(_pin, LOW);
    unsigned long totalTime = highTime + lowTime;

    if (totalTime == 0)
        return _lastPosition;

    float    dutyCycle = (float)highTime / totalTime;
    uint16_t maxPos    = _is12Bit ? 4095 : 1023;

    return (uint16_t)(dutyCycle * maxPos);
}

float MAE3Encoder::positionToAngle(uint16_t position) {
    uint16_t maxPos = _is12Bit ? 4095 : 1023;
    return (float)position * 360.0f / maxPos;
}