#pragma once

namespace MotorSpecs {
    // Electrical Specifications
    struct Electrical {
        static constexpr float   RATED_VOLTAGE    = 1.75f;  // Rated voltage (V)
        static constexpr float   RATED_CURRENT    = 0.5f;   // Rated current per phase (A)
        static constexpr float   PHASE_RESISTANCE = 5.5f;   // Resistance per phase (Ohm) ±10%
        static constexpr float   PHASE_INDUCTANCE = 0.9f;   // Inductance per phase (mH) ±20%
        static constexpr uint8_t PHASE_COUNT      = 2;      // Number of phases
    };

    // Mechanical Specifications
    struct Mechanical {
        static constexpr float STEP_ANGLE     = 1.8f;   // Step angle (degrees)
        static constexpr float HOLDING_TORQUE = 16.0f;  // Holding torque (N.cm)
        static constexpr float DETENT_TORQUE  = 2.0f;   // Detent torque (N.cm)
        static constexpr float ROTOR_INERTIA  = 2.0f;   // Rotor inertia (g.cm²)
        static constexpr float MASS           = 30.0f;  // Mass (g)
    };

    // Thermal Specifications
    struct Thermal {
        // Passive cooling specifications
        static constexpr bool HAS_ACTIVE_COOLING = false;  // No cooling fan/heatsink

        // Thermal time constants
        static constexpr float THERMAL_TIME_CONSTANT =
            600.0f;  // Motor thermal time constant (seconds)
        static constexpr float THERMAL_RESISTANCE =
            4.5f;  // Motor to ambient thermal resistance (°C/W)
        static constexpr float THERMAL_CAPACITY = 450.0f;  // Motor thermal capacity (J/°C)

        // Duty cycle limitations for passive cooling
        static constexpr float MAX_CONTINUOUS_DUTY = 0.7f;  // Maximum continuous duty cycle (70%)
        static constexpr float RECOMMENDED_DUTY    = 0.5f;  // Recommended duty cycle (50%)
        static constexpr uint32_t MIN_REST_TIME =
            300;  // Minimum rest time between operations (seconds)

        // Temperature rise characteristics
        static constexpr float MAX_TEMP_RISE =
            55.0f;  // Maximum temperature rise above ambient (°C)
        static constexpr float TEMP_RISE_PER_WATT =
            2.5f;  // Temperature rise per watt dissipated (°C/W)

        // Cooling parameters
        struct PassiveCooling {
            static constexpr float NATURAL_COOLING_RATE = 0.05f;  // Natural cooling rate (°C/s)
            static constexpr float SURFACE_AREA         = 50.0f;  // Motor surface area (cm²)
            static constexpr float EMISSIVITY           = 0.9f;   // Surface emissivity
        };

        // Thermal protection settings
        struct Protection {
            // Temperature thresholds for passive cooling
            static constexpr float ABSOLUTE_MAX_TEMP = 85.0f;  // Absolute maximum temperature (°C)
            static constexpr float REDUCED_PERFORMANCE_TEMP =
                70.0f;  // Temperature for reduced performance (°C)
            static constexpr float RECOVERY_TEMP =
                60.0f;  // Temperature to resume normal operation (°C)

            // Duty cycle management
            static constexpr float DUTY_REDUCTION_PER_DEGREE =
                0.05f;  // Duty reduction per °C above warning
            static constexpr uint32_t MIN_COOLDOWN_TIME =
                600;  // Minimum cooldown time after thermal warning (seconds)

            // Current derating
            static constexpr float CURRENT_DERATING_START =
                65.0f;  // Temperature to start current derating (°C)
            static constexpr float CURRENT_DERATING_FACTOR =
                0.02f;  // Current reduction per °C above derating start
        };
    };

    // Protection Limits
    struct Protection {
        // Temperature limits
        static constexpr float MAX_TEMP         = 80.0f;   // Maximum operating temperature (°C)
        static constexpr float MIN_TEMP         = -20.0f;  // Minimum operating temperature (°C)
        static constexpr float THERMAL_SHUTDOWN = 85.0f;   // Thermal shutdown temperature (°C)
        static constexpr float TEMP_WARNING     = 70.0f;   // Temperature warning threshold (°C)

        // Electrical limits
        static constexpr float MAX_VOLTAGE = 2.0f;  // Maximum voltage (V)
        static constexpr float MAX_CURRENT = 0.6f;  // Maximum current per phase (A)
        static constexpr float MIN_VOLTAGE = 1.5f;  // Minimum voltage (V)

        // Mechanical limits
        static constexpr float MAX_SPEED        = 1000.0f;  // Maximum speed (steps/second)
        static constexpr float MAX_ACCELERATION = 5000.0f;  // Maximum acceleration (steps/second²)

        // Updated thermal protection for passive cooling
        static constexpr float MAX_CONTINUOUS_TEMP =
            70.0f;  // Maximum temperature for continuous operation (°C)
        static constexpr float RECOVERY_HYSTERESIS =
            10.0f;  // Temperature drop required before resuming (°C)

        // Duty cycle protection
        static constexpr uint32_t MAX_RUN_TIME  = 1800;  // Maximum continuous run time (seconds)
        static constexpr uint32_t MIN_COOL_TIME = 300;   // Minimum cooling time (seconds)
    };

    // Operating Conditions
    struct Operating {
        // Environmental conditions
        static constexpr float AMBIENT_TEMP = 25.0f;  // Normal ambient temperature (°C)
        static constexpr float HUMIDITY_MAX = 85.0f;  // Maximum humidity (%)
        static constexpr float HUMIDITY_MIN = 20.0f;  // Minimum humidity (%)

        // Insulation specifications
        static constexpr char  INSULATION_CLASS      = 'B';     // Insulation class
        static constexpr float INSULATION_RESISTANCE = 100.0f;  // Insulation resistance (MΩ)

        // Timing specifications
        static constexpr float MIN_PULSE_WIDTH = 2.0f;  // Minimum step pulse width (µs)
        static constexpr float DIR_SETUP_TIME  = 1.0f;  // Direction setup time (µs)
        static constexpr float DIR_HOLD_TIME   = 1.0f;  // Direction hold time (µs)
    };

    // Performance Monitoring
    struct Monitoring {
        static constexpr bool ENABLE_TEMP_MONITORING    = true;  // Enable temperature monitoring
        static constexpr bool ENABLE_CURRENT_MONITORING = true;  // Enable current monitoring
        static constexpr bool ENABLE_STALL_DETECTION    = true;  // Enable stall detection

        // Monitoring thresholds
        static constexpr float    STALL_THRESHOLD       = 0.8f;   // Stall detection threshold
        static constexpr float    OVERCURRENT_THRESHOLD = 0.55f;  // Overcurrent threshold (A)
        static constexpr uint16_t MONITORING_INTERVAL   = 100;    // Monitoring interval (ms)

        // Thermal monitoring for passive cooling
        struct ThermalMonitoring {
            static constexpr uint16_t TEMP_CHECK_INTERVAL =
                1000;  // Temperature check interval (ms)
            static constexpr uint16_t DUTY_CHECK_INTERVAL = 5000;  // Duty cycle check interval (ms)
            static constexpr float    TEMP_RISE_THRESHOLD =
                2.0f;  // Maximum temperature rise rate (°C/s)
            static constexpr uint8_t TEMP_HISTORY_SIZE =
                10;  // Number of temperature readings to keep
        };
    };
}  // namespace MotorSpecs