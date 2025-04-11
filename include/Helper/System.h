#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdint.h>

namespace MotionSystem
{
    namespace System
    {
        static const uint8_t  NUM_MOTORS                = 4;
        static const uint8_t  NUM_PWM_ENCODERS          = 4;
        static const uint8_t  NUM_DIFF_ENCODERS         = 0;
        static const uint32_t SERIAL_BAUD_RATE          = 115200;  // Serial communication speed
        static const uint32_t STARTUP_DELAY_MS          = 1000;    // Initial delay for system stability
        constexpr uint8_t     MICROSTEPS                = 16;      // Microstepping configuration
        constexpr uint16_t    STEPS_PER_REV             = 200;     // Motor steps per revolution
        constexpr uint16_t    ENCODER_PPR               = 1000;    // Pulses per revolution (4000 CPR with quadrature)
        constexpr float       LEAD_SCREW_PITCH          = 0.5f;    // Lead screw pitch in mm
        constexpr float       PIXEL_SIZE                = 5.2f;    // Size of one pixel in micrometers
        constexpr float       TOTAL_TRAVEL_MM           = 30.0f;   // Total travel distance in mm
        constexpr float       TOTAL_TRAVEL_MICRONS      = TOTAL_TRAVEL_MM * 1000.0f;
        constexpr float       REL_TRAVEL_LIMIT_MM       = 3.0f;  // Relative travel limit in mm
        constexpr float       REL_TRAVEL_LIMIT_MICRONS  = REL_TRAVEL_LIMIT_MM * 1000.0f;
        constexpr uint16_t    STATUS_UPDATE_MS          = 400;  // Status update interval in milliseconds
        constexpr float       MOTOR_STEPS_PER_MICRON    = (STEPS_PER_REV * MICROSTEPS) / (LEAD_SCREW_PITCH * 1000.0f);
        constexpr float       ENCODER_COUNTS_PER_MICRON = (ENCODER_PPR * 4.0f) / (LEAD_SCREW_PITCH * 1000.0f);
    }  // namespace System
}  // namespace MotionSystem

#endif  // SYSTEM_H