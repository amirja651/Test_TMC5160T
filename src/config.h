#pragma once

#include <Arduino.h>
#include "motor_specs.h"

namespace Config {
    // SPI Pins for ESP32 WROOM
    struct SPIPins {
        static constexpr uint8_t MOSI = 23;  // VSPI MOSI
        static constexpr uint8_t MISO = 19;  // VSPI MISO
        static constexpr uint8_t SCK  = 18;  // VSPI SCK
        static constexpr uint8_t CS   = 5;   // VSPI CS
    };

    // Motor control pins
    struct MotorPins {
        static constexpr uint8_t STEP = 4;   // GPIO4
        static constexpr uint8_t DIR  = 2;   // GPIO2
        static constexpr uint8_t EN   = 15;  // GPIO15
    };

    // Motor driver parameters (updated)
    struct MotorParams {
        // Current settings
        static constexpr uint16_t CURRENT_MA =
            500;  // Motor RMS current in mA (based on rated current)
        static constexpr uint8_t MICROSTEPS = 16;  // Microstep resolution

        // Protection settings
        static constexpr float MAX_TEMP_THRESHOLD = MotorSpecs::Protection::TEMP_WARNING;
        static constexpr float MAX_CURRENT_THRESHOLD =
            MotorSpecs::Protection::MAX_CURRENT * 1000;  // Convert to mA

        // Performance settings
        static constexpr uint32_t MAX_SPEED    = 1000;  // Maximum speed in steps/second
        static constexpr uint32_t ACCELERATION = 500;   // Acceleration in steps/second²
    };

    // Timing constants
    struct Timing {
        static constexpr uint32_t SERIAL_BAUDRATE = 115200;
        static constexpr uint32_t INIT_DELAY_MS   = 1000;
    };

    // Protection and monitoring settings
    struct Protection {
        static constexpr bool     ENABLE_STALLGUARD          = true;
        static constexpr bool     ENABLE_TEMPERATURE_WARNING = true;
        static constexpr bool     ENABLE_SHORT_PROTECTION    = true;
        static constexpr uint16_t MONITORING_INTERVAL_MS =
            MotorSpecs::Monitoring::MONITORING_INTERVAL;

        // Thermal protection
        static constexpr bool  ENABLE_THERMAL_PROTECTION    = true;
        static constexpr bool  ENABLE_DUTY_CYCLE_MONITORING = true;
        static constexpr float MAX_TEMP_RISE_RATE =
            MotorSpecs::Monitoring::ThermalMonitoring::TEMP_RISE_THRESHOLD;
    };

    // Motor thermal management
    struct ThermalManagement {
        // Operation timing
        static constexpr uint32_t MAX_CONTINUOUS_RUN_TIME =
            MotorSpecs::Protection::MAX_RUN_TIME * 1000;  // Convert to milliseconds

        // Cooling parameters
        static constexpr float DUTY_CYCLE =
            MotorSpecs::Thermal::RECOMMENDED_DUTY;  // Default duty cycle

        // Temperature monitoring
        static constexpr uint16_t TEMP_CHECK_INTERVAL =
            MotorSpecs::Monitoring::ThermalMonitoring::TEMP_CHECK_INTERVAL;

        // Current derating
        static constexpr bool  ENABLE_CURRENT_DERATING = true;
        static constexpr float DERATING_START_TEMP =
            MotorSpecs::Thermal::Protection::CURRENT_DERATING_START;
    };
}  // namespace Config
