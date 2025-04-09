#ifndef MAE3_ENCODER_H
#define MAE3_ENCODER_H

#include <Arduino.h>

namespace MotionSystem
{
    class PWMEncoder
    {
    public:
        PWMEncoder(uint8_t signalPin);
        void     begin();
        bool     update();
        float    getPositionDegrees() const;
        uint32_t getPulseWidth() const;

    private:
        static void               handleInterrupt();
        void                      measurePulse();
        const uint8_t             signalPin;
        uint32_t                  lastPulseWidth;
        float                     lastPosition;
        unsigned long             lastUpdateTime;
        static PWMEncoder*        instance;
        volatile unsigned long    pulseStartTime;
        volatile unsigned long    currentPulseWidth;
        volatile bool             newPulseAvailable;
        static constexpr uint32_t MIN_PULSE_WIDTH    = 5;
        static constexpr uint32_t MAX_PULSE_WIDTH    = 3935;
        static constexpr float    POSITION_THRESHOLD = 0.5f;
    };
}  // namespace MotionSystem

#endif  // MAE3_ENCODER_H