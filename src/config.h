#ifndef CONFIG_H
#define CONFIG_H

#include "esp32_pins.h"

namespace Config {
    // SPI Configuration
    struct SPI {
        static const uint8_t MOSI = ESP32Pins::RightSide::GPIO23;  // MOSI, V_SPI_D
        static const uint8_t MISO = ESP32Pins::RightSide::GPIO19;  // MISO, V_SPI_Q
        static const uint8_t SCK  = ESP32Pins::RightSide::GPIO18;  // SCK, V_SPI_CLK
        static const uint8_t CS   = ESP32Pins::RightSide::GPIO5;   // V_SPI_CS0
    };

    // TMC5160 Motor Control Configuration
    struct TMC5160T_Driver {
        static const uint8_t STEP_PIN = ESP32Pins::RightSide::GPIO4;   // ADC2_0, Touch0
        static const uint8_t DIR_PIN  = ESP32Pins::RightSide::GPIO2;   // ADC2_2, Touch2
        static const uint8_t EN_PIN   = ESP32Pins::RightSide::GPIO15;  // ADC2_3, Touch3

        // Motor driver settings
        static const uint16_t TMC_CURRENT_MA = 1000;  // Motor RMS current in mA
        static const uint8_t  MICROSTEPS     = 16;    // Microstep resolution
    };
};  // namespace Config

#endif  // CONFIG_H