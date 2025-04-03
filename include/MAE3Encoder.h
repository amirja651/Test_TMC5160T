#ifndef MAE3_ENCODER_H
#define MAE3_ENCODER_H

#include <Arduino.h>

class MAE3Encoder {
public:
    MAE3Encoder(uint8_t pin, bool is12Bit = false);

    void     begin();
    uint16_t readPosition();
    float    readAngle();

private:
    uint8_t  _pin;
    bool     _is12Bit;
    uint16_t _lastPosition;

    uint16_t readPWM();
    float    positionToAngle(uint16_t position);
};

#endif  // MAE3_ENCODER_H