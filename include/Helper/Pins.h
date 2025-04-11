#ifndef PINS_H
#define PINS_H

#include "Helper/ESP32Pins.h"

namespace MotionSystem
{
    namespace Pins
    {
        namespace SPI
        {
            static const uint8_t MOSI = ESP32Pins::RightSide::GPIO23;
            static const uint8_t MISO = ESP32Pins::RightSide::GPIO19;
            static const uint8_t SCK  = ESP32Pins::RightSide::GPIO18;

            static const uint8_t MOTOR1_CS = ESP32Pins::RightSide::GPIO4;
            static const uint8_t MOTOR2_CS = ESP32Pins::RightSide::GPIO5;
            static const uint8_t MOTOR3_CS = ESP32Pins::RightSide::GPIO15;
            static const uint8_t MOTOR4_CS = ESP32Pins::RightSide::GPIO16;
        }  // namespace SPI

        namespace TMC5160T_Driver
        {
            static const uint8_t MOTOR1_EN_PIN = ESP32Pins::RightSide::GPIO17;
            static const uint8_t MOTOR2_EN_PIN = ESP32Pins::LeftSide::GPIO25;
            static const uint8_t MOTOR3_EN_PIN = ESP32Pins::RightSide::GPIO21;
            static const uint8_t MOTOR4_EN_PIN = ESP32Pins::RightSide::GPIO22;

            static const uint8_t MOTOR1_STEP_PIN = ESP32Pins::LeftSide::GPIO26;
            static const uint8_t MOTOR2_STEP_PIN = ESP32Pins::LeftSide::GPIO27;
            static const uint8_t MOTOR3_STEP_PIN = ESP32Pins::LeftSide::GPIO32;
            static const uint8_t MOTOR4_STEP_PIN = ESP32Pins::LeftSide::GPIO33;

            static const uint8_t MOTOR1_DIR_PIN = ESP32Pins::LeftSide::GPIO12;
            static const uint8_t MOTOR2_DIR_PIN = ESP32Pins::LeftSide::GPIO13;
            static const uint8_t MOTOR3_DIR_PIN = ESP32Pins::LeftSide::GPIO14;
            static const uint8_t MOTOR4_DIR_PIN = ESP32Pins::RightSide::GPIO2;
        }  // namespace TMC5160T_Driver

        namespace Simple_Driver
        {
            constexpr uint8_t STEP_PIN   = ESP32Pins::LeftSide::GPIO32;
            constexpr uint8_t DIR_PIN    = ESP32Pins::LeftSide::GPIO33;
            constexpr uint8_t ENABLE_PIN = ESP32Pins::LeftSide::GPIO14;
        }  // namespace Simple_Driver

        namespace Encoder
        {
            namespace Differential
            {
                constexpr uint8_t ENCODER_A_PIN     = ESP32Pins::RightSide::GPIO22;
                constexpr uint8_t ENCODER_B_PIN     = ESP32Pins::RightSide::GPIO23;
                constexpr uint8_t ENCODER_INDEX_PIN = ESP32Pins::RightSide::GPIO21;
            }  // namespace Differential

            namespace PWM
            {
                constexpr uint8_t ENCODER_P1_PIN = ESP32Pins::LeftSide::GPIO35;
                constexpr uint8_t ENCODER_P2_PIN = ESP32Pins::LeftSide::GPIO35;
                constexpr uint8_t ENCODER_P3_PIN = ESP32Pins::LeftSide::GPIO35;
                constexpr uint8_t ENCODER_P4_PIN = ESP32Pins::LeftSide::GPIO35;
            }  // namespace PWM
        }  // namespace Encoder

        namespace LimitSwitch
        {
            constexpr uint8_t LIMIT_SWITCH_PIN = ESP32Pins::LeftSide::GPIO13;
        }

    }  // namespace Pins
}  // namespace MotionSystem

#endif  // PINS_H