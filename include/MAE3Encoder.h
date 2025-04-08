#ifndef MAE3_ENCODER_H
#define MAE3_ENCODER_H

#include <Arduino.h>

namespace MotionSystem
{
    class MAE3Encoder
    {
    public:
        /**
         * @brief Constructor for MAE3Encoder
         * @param signalPin GPIO pin connected to encoder PWM output
         */
        MAE3Encoder(uint8_t signalPin);

        /**
         * @brief Initialize the encoder
         */
        void begin();

        /**
         * @brief Read and update the current position
         * @return true if position changed, false otherwise
         */
        bool update();

        /**
         * @brief Get the current position in degrees (0-360)
         * @return Current position in degrees
         */
        float getPositionDegrees() const;

        /**
         * @brief Get the raw pulse width in microseconds
         * @return Pulse width in microseconds
         */
        uint32_t getPulseWidth() const;

    private:
        static void handleInterrupt();  // Static interrupt handler
        void        measurePulse();     // Measure pulse width from interrupt

        const uint8_t signalPin;       // GPIO pin for encoder signal
        uint32_t      lastPulseWidth;  // Last measured pulse width
        float         lastPosition;    // Last calculated position
        unsigned long lastUpdateTime;  // Last update timestamp

        // Interrupt-related members
        static MAE3Encoder*    instance;           // Static instance pointer for interrupt handler
        volatile unsigned long pulseStartTime;     // Start time of current pulse
        volatile unsigned long currentPulseWidth;  // Current pulse width measurement
        volatile bool          newPulseAvailable;  // Flag indicating new pulse measurement

        // Constants for pulse width filtering
        static constexpr uint32_t MIN_PULSE_WIDTH    = 5;     // Minimum valid pulse width in microseconds
        static constexpr uint32_t MAX_PULSE_WIDTH    = 3935;  // Maximum valid pulse width in microseconds
        static constexpr float    POSITION_THRESHOLD = 0.5f;  // Minimum position change in degrees
    };
}  // namespace MotionSystem

#endif  // MAE3_ENCODER_H