#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdint.h>

namespace ESP32Pins
{
    // Left Side Pins (Top to Bottom)
    struct LeftSide
    {
        // Input only pins
        static const uint8_t GPIO36 = 36;  // Input only, ADC1_0
        static const uint8_t GPIO39 = 39;  // Input only, ADC1_3
        static const uint8_t GPIO34 = 34;  // Input only, ADC1_6
        static const uint8_t GPIO35 = 35;  // Input only, ADC1_7
        // Regular GPIO pins
        static const uint8_t GPIO32 = 32;  // ADC1_4, Touch9, XTAL_32K_P
        static const uint8_t GPIO33 = 33;  // ADC1_5, Touch8, XTAL_32K_N
        static const uint8_t GPIO25 = 25;  // ADC2_8, DAC1
        static const uint8_t GPIO26 = 26;  // ADC2_9, DAC2
        static const uint8_t GPIO27 = 27;  // ADC2_7, Touch7
        static const uint8_t GPIO14 = 14;  // ADC2_6, Touch6
        static const uint8_t GPIO12 = 12;  // ADC2_5, Touch5
        static const uint8_t GPIO13 = 13;  // ADC2_4, Touch4
    };

    // Right Side Pins (Top to Bottom)
    struct RightSide
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
    };
};  // namespace ESP32Pins

namespace Config
{
    // System Configuration
    struct System
    {
        static const uint32_t SERIAL_BAUD_RATE = 115200;  // Serial communication speed
        static const uint32_t STARTUP_DELAY_MS = 1000;    // Initial delay for system stability
    };

    // SPI Configuration
    struct SPI
    {
        static const uint8_t MOSI = ESP32Pins::RightSide::GPIO23;
        static const uint8_t MISO = ESP32Pins::RightSide::GPIO19;
        static const uint8_t SCK  = ESP32Pins::RightSide::GPIO18;

        static const uint8_t MOTOR1_CS = ESP32Pins::RightSide::GPIO4;
        static const uint8_t MOTOR2_CS = ESP32Pins::RightSide::GPIO5;
        static const uint8_t MOTOR3_CS = ESP32Pins::RightSide::GPIO15;
        static const uint8_t MOTOR4_CS = ESP32Pins::RightSide::GPIO16;
    };

    // TMC5160 Motor Control Configuration
    struct TMC5160T_Driver
    {
        // Number of motors in the system
        static const uint8_t NUM_MOTORS = 4;

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
    };

    // Command Handler Configuration
    namespace CommandHandler
    {
        constexpr char CMD_FORWARD     = 'w';  // *
        constexpr char CMD_REVERSE     = 's';  // *
        constexpr char CMD_STOP        = 'x';  // *
        constexpr char CMD_RESET       = 'z';
        constexpr char CMD_TEST_SPI    = 't';  // *
        constexpr char CMD_SHOW_STATUS = 'i';  // *
        constexpr char CMD_SHOW_CONFIG = 'p';  // *
        constexpr char CMD_SHOW_TEMP   = 'm';
        constexpr char CMD_TOGGLE_MODE = 'n';
        constexpr char CMD_HELP        = 'h';  // *
        constexpr char CMD_HELP_ALT    = '?';  // *
    }  // namespace CommandHandler

};  // namespace Config

#endif  // CONFIG_H