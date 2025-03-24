#ifndef MOTOR_SPECS_H
#define MOTOR_SPECS_H

#include <stdint.h>

struct MotorSpecs {
    // General specifications
    struct General {
        const float   stepAngle            = 1.8f;  // degrees
        const uint8_t numberOfPhases       = 2;
        const float   insulationResistance = 100.0f;  // MΩ min at 500V DC
        const char    insulationClass      = 'B';
        const float   rotorInertia         = 2.0f;   // g·cm²
        const float   mass                 = 30.0f;  // grams
    } general;

    // Motor constraints for pancake type
    struct Constraints {
        const float maxTemperature  = 50.0f;   // default: 80 °C (Class B insulation limit)
        const float maxCurrent      = 0.55f;   // A (10% safety margin)
        const float minVoltage      = 2.0f;    // V (minimum operating voltage)
        const float maxAcceleration = 500.0f;  // default: 1000 steps/s² (for smooth operation)
        const float maxSpeed        = 250.0f;  // default: 500 steps/s (for stable operation)
    } constraints;

    // Operational parameters
    struct Operation {
        const uint16_t stepsPerRev      = 200;     // steps per revolution
        const float    recommendedSpeed = 200.0f;  // steps/s
        const float    startupCurrent   = 0.3f;    // A (reduced current for startup)
        const float    idleCurrent      = 0.25f;   // A (reduced current when idle)
    } operation;

    // Electrical specifications
    struct Electrical {
        const float ratedVoltage    = 7.25f;  // V
        const float ratedCurrent    = 0.5f;   // A
        const float phaseResistance = 3.5f;   // Ω ±10%
        const float phaseInductance = 0.90f;  // mH ±20%
        const float holdingTorque   = 16.0f;  // mN·m
        const float detentTorque    = 2.0f;   // mN·m
    } electrical;
};

// Create a global instance of motor specifications
static const MotorSpecs MOTOR_P28SHD4611_12SK;

#endif  // MOTOR_SPECS_H