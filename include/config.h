#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdint.h>

namespace MotionSystem
{  // namespace ESP32Pins
    namespace Types
    {
        using Timestamp        = uint64_t;
        using EncoderPosition  = int32_t;
        using MicronPosition   = float;
        using PixelPosition    = float;
        using StepPosition     = int32_t;
        using Speed            = float;
        using Acceleration     = float;
        using StatusCallback   = std::function<void(bool)>;
        using CompleteCallback = std::function<void(void)>;

        enum class ErrorCode
        {
            SUCCESS = 0,
            LIMIT_SWITCH_TRIGGERED,
            POSITION_OUT_OF_BOUNDS,
            CALIBRATION_FAILED,
            TIMEOUT
        };
    }  // namespace Types

    // namespace ESP32Pins
    namespace ESP32Pins
    {
        namespace LeftSide
        {
            static const uint8_t GPIO36 = 36;  // Input only, ADC1_0
            static const uint8_t GPIO39 = 39;  // Input only, ADC1_3
            static const uint8_t GPIO34 = 34;  // Input only, ADC1_6
            static const uint8_t GPIO35 = 35;  // Input only, ADC1_7
            static const uint8_t GPIO32 = 32;  // ADC1_4, Touch9, XTAL_32K_P
            static const uint8_t GPIO33 = 33;  // ADC1_5, Touch8, XTAL_32K_N
            static const uint8_t GPIO25 = 25;  // ADC2_8, DAC1
            static const uint8_t GPIO26 = 26;  // ADC2_9, DAC2
            static const uint8_t GPIO27 = 27;  // ADC2_7, Touch7
            static const uint8_t GPIO14 = 14;  // ADC2_6, Touch6
            static const uint8_t GPIO12 = 12;  // ADC2_5, Touch5
            static const uint8_t GPIO13 = 13;  // ADC2_4, Touch4
        }  // namespace LeftSide

        namespace RightSide
        {
            static const uint8_t GPIO23 = 23;  // MOSI, V_SPI_D
            static const uint8_t GPIO22 = 22;  // V_SPI_WP
            static const uint8_t GPIO1  = 1;   // TX0
            static const uint8_t GPIO3  = 3;   // RX0
            static const uint8_t GPIO21 = 21;  // V_SPI_HD
            static const uint8_t GPIO19 = 19;  // MISO, V_SPI_Q
            static const uint8_t GPIO18 = 18;  // SCK, V_SPI_CLK
            static const uint8_t GPIO5  = 5;   // V_SPI_CS0
            static const uint8_t GPIO17 = 17;  // TX2
            static const uint8_t GPIO16 = 16;  // RX2
            static const uint8_t GPIO4  = 4;   // ADC2_0, Touch0
            static const uint8_t GPIO2  = 2;   // ADC2_2, Touch2
            static const uint8_t GPIO15 = 15;  // ADC2_3, Touch3
        }  // namespace RightSide
    }  // namespace ESP32Pins

    namespace Config
    {
        namespace SPI
        {
            static const uint8_t MOSI      = ESP32Pins::RightSide::GPIO23;
            static const uint8_t MISO      = ESP32Pins::RightSide::GPIO19;
            static const uint8_t SCK       = ESP32Pins::RightSide::GPIO18;
            static const uint8_t MOTOR1_CS = ESP32Pins::RightSide::GPIO4;
            static const uint8_t MOTOR2_CS = ESP32Pins::RightSide::GPIO5;
            static const uint8_t MOTOR3_CS = ESP32Pins::RightSide::GPIO15;
            static const uint8_t MOTOR4_CS = ESP32Pins::RightSide::GPIO16;
        }  // namespace SPI

        namespace TMC5160T_Driver
        {
            static const uint8_t NUM_MOTORS        = 4;
            static const uint8_t NUM_PWM_ENCODERS  = 4;
            static const uint8_t NUM_DIFF_ENCODERS = 0;
            static const uint8_t MOTOR1_EN_PIN     = ESP32Pins::RightSide::GPIO17;
            static const uint8_t MOTOR2_EN_PIN     = ESP32Pins::LeftSide::GPIO25;
            static const uint8_t MOTOR3_EN_PIN     = ESP32Pins::RightSide::GPIO21;
            static const uint8_t MOTOR4_EN_PIN     = ESP32Pins::RightSide::GPIO22;
            static const uint8_t MOTOR1_STEP_PIN   = ESP32Pins::LeftSide::GPIO26;
            static const uint8_t MOTOR2_STEP_PIN   = ESP32Pins::LeftSide::GPIO27;
            static const uint8_t MOTOR3_STEP_PIN   = ESP32Pins::LeftSide::GPIO32;
            static const uint8_t MOTOR4_STEP_PIN   = ESP32Pins::LeftSide::GPIO33;
            static const uint8_t MOTOR1_DIR_PIN    = ESP32Pins::LeftSide::GPIO12;
            static const uint8_t MOTOR2_DIR_PIN    = ESP32Pins::LeftSide::GPIO13;
            static const uint8_t MOTOR3_DIR_PIN    = ESP32Pins::LeftSide::GPIO14;
            static const uint8_t MOTOR4_DIR_PIN    = ESP32Pins::RightSide::GPIO2;
        }  // namespace TMC5160T_Driver

        namespace Pins
        {
            constexpr uint8_t STEP_PIN          = 32;
            constexpr uint8_t DIR_PIN           = 33;
            constexpr uint8_t ENABLE_PIN        = 14;
            constexpr uint8_t ENCODER_A_PIN     = 22;
            constexpr uint8_t ENCODER_B_PIN     = 23;
            constexpr uint8_t ENCODER_P1_PIN    = 35;
            constexpr uint8_t ENCODER_P2_PIN    = 35;
            constexpr uint8_t ENCODER_P3_PIN    = 35;
            constexpr uint8_t ENCODER_P4_PIN    = 35;
            constexpr uint8_t ENCODER_INDEX_PIN = 21;
            constexpr uint8_t LIMIT_SWITCH_PIN  = 13;
        }  // namespace Pins

        namespace System
        {
            static const uint32_t SERIAL_BAUD_RATE         = 115200;  // Serial communication speed
            static const uint32_t STARTUP_DELAY_MS         = 1000;    // Initial delay for system stability
            constexpr uint8_t     MICROSTEPS               = 16;      // Microstepping configuration
            constexpr uint16_t    STEPS_PER_REV            = 200;     // Motor steps per revolution
            constexpr uint16_t    ENCODER_PPR              = 1000;   // Pulses per revolution (4000 CPR with quadrature)
            constexpr float       LEAD_SCREW_PITCH         = 0.5f;   // Lead screw pitch in mm
            constexpr float       PIXEL_SIZE               = 5.2f;   // Size of one pixel in micrometers
            constexpr float       TOTAL_TRAVEL_MM          = 30.0f;  // Total travel distance in mm
            constexpr float       TOTAL_TRAVEL_MICRONS     = TOTAL_TRAVEL_MM * 1000.0f;
            constexpr float       REL_TRAVEL_LIMIT_MM      = 3.0f;  // Relative travel limit in mm
            constexpr float       REL_TRAVEL_LIMIT_MICRONS = REL_TRAVEL_LIMIT_MM * 1000.0f;
            constexpr uint16_t    STATUS_UPDATE_MS         = 400;  // Status update interval in milliseconds
            constexpr float       MOTOR_STEPS_PER_MICRON = (STEPS_PER_REV * MICROSTEPS) / (LEAD_SCREW_PITCH * 1000.0f);
            constexpr float       ENCODER_COUNTS_PER_MICRON = (ENCODER_PPR * 4.0f) / (LEAD_SCREW_PITCH * 1000.0f);
        }  // namespace System

        namespace Motion
        {
            constexpr uint16_t MAX_SPEED       = 5000;   // Maximum step frequency in Hz
            constexpr uint16_t ACCELERATION    = 10000;  // Steps per second per second
            constexpr uint16_t PID_UPDATE_FREQ = 1000;   // PID update frequency in Hz
        }  // namespace Motion

        namespace PID
        {
            constexpr float    KP           = 1.2f;   // 0.8f   // Proportional gain
            constexpr float    KI           = 0.15f;  // 0.1f;  // Integral gain
            constexpr float    KD           = 0.08f;  // 0.05f  // Derivative gain
            constexpr uint16_t MAX_INTEGRAL = 1000;   // Anti-windup limit
        }  // namespace PID

        namespace Tasks
        {
            constexpr uint16_t PID_TASK_STACK_SIZE    = 4096;
            constexpr uint8_t  PID_TASK_PRIORITY      = 3;
            constexpr uint8_t  PID_TASK_CORE          = 1;
            constexpr uint16_t MOTION_TASK_STACK_SIZE = 4096;
            constexpr uint8_t  MOTION_TASK_PRIORITY   = 2;
            constexpr uint8_t  MOTION_TASK_CORE       = 1;
            constexpr uint16_t STATUS_TASK_STACK_SIZE = 4096;
            constexpr uint8_t  STATUS_TASK_PRIORITY   = 1;
            constexpr uint8_t  STATUS_TASK_CORE       = 0;
        }  // namespace Tasks

    }  // namespace Config
}  // namespace MotionSystem

#endif  // CONFIG_H