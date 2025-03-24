#pragma once
#include <TMCStepper.h>
#include "config.h"

class MotorController {
public:
    // Status Structures
    struct DriverStatus {
        uint8_t  chopperMode;  // 0: SpreadCycle, 1: StealthChop
        bool     enabled;
        uint16_t microsteps;
        uint16_t currentSetting;  // mA
    };

    struct MotorStatus {
        int32_t currentPosition;  // steps
        int32_t targetPosition;   // steps
        float   currentSpeed;     // steps/sec
        uint8_t runMode;          // 0: Stop, 1: Position, 2: Velocity, 3: Hold
        float   motorLoad;        // 0-100%
    };

    struct ThermalStatus {
        float   currentTemp;        // °C
        float   warningTemp;        // °C
        float   maxTemp;            // °C
        float   driverTemp;         // °C
        float   driverWarningTemp;  // °C
        float   driverMaxTemp;      // °C
        bool    protectionActive;
        uint8_t protectionLevel;  // 0-100%
    };

    struct PowerStatus {
        float    supplyVoltage;  // V
        uint16_t motorCurrent;   // mA
        float    efficiency;     // 0-100%
    };

    struct ErrorFlags {
        bool overtemperature;
        bool shortCircuit;
        bool openLoad;
        bool overCurrent;
        bool stallGuard;

        bool any() const {
            return overtemperature || shortCircuit || openLoad || overCurrent || stallGuard;
        }
    };

    struct StepperConfig {
        uint16_t microsteps;
        uint8_t  currentScale;  // 0-100%
        uint16_t holdCurrent;   // mA
        uint16_t runCurrent;    // mA
    };

    // Constructor and basic methods
    MotorController();
    ~MotorController() = default;

    // Public interface
    bool initialize();
    bool configure();
    bool isReady() const {
        return isInitialized && isConfigured;
    }

    // Motor control methods
    bool enable();
    bool disable();
    bool setSpeed(uint32_t speed);
    bool setDirection(bool clockwise);
    bool moveToPosition(int32_t targetPosition);

    // Status and monitoring
    float    getCurrentTemperature() const;
    uint32_t getCurrentSpeed() const;
    bool     isEnabled() const;

    // Status methods
    DriverStatus  getDriverStatus() const;
    MotorStatus   getMotorStatus() const;
    ThermalStatus getThermalStatus() const;
    PowerStatus   getPowerStatus() const;
    ErrorFlags    getErrorFlags() const;
    StepperConfig getStepperConfig() const;

    // Configuration getters
    uint16_t getMicrostepResolution() const;
    uint16_t getCurrentSetting() const;

    // SPI test interface
    struct SPITestResult {
        bool     success;
        uint32_t version;
        String   errorMessage;
    };

    SPITestResult testConnection();
    bool          isSPIConnected() const;

    // New methods
    bool setMotorCurrent(uint16_t current);
    bool emergencyStop();

    // Temperature monitoring
    float getTemperature();
    bool  overTemperaturePreWarning();
    bool  overTemperatureFlag();

    // SPI test methods
    bool testVersionMatch();
    bool testRegisterAccess();
    bool testErrorStatus();
    bool testSPICommunication();

    // New methods
    void updateThermalStatus();

    // Auto-tuning methods
    bool     autoTune();
    uint16_t findOptimalCurrent();
    uint16_t findOptimalMicrosteps();
    uint32_t findOptimalAcceleration();
    uint32_t findOptimalVelocity();
    void     tuneStealthChop();
    float    measureMovementSmoothness();

private:
    TMC5160Stepper driver;
    bool           isInitialized;
    bool           isConfigured;

    // Internal status tracking
    DriverStatus  driverStatus;
    MotorStatus   motorStatus;
    ThermalStatus thermalStatus;
    PowerStatus   powerStatus;
    ErrorFlags    errorFlags;
    StepperConfig stepperConfig;

    // Private methods for internal operations
    void initializePins();
    void initializeSPICommunication();
    bool configureDriverSettings();
    bool verifyDriverSettings() const;
    void handleError(const char* message);

    // Status update methods
    void updateDriverStatus();
    void updateMotorStatus();
    void updatePowerStatus();
    void updateErrorFlags();
    void updateStepperConfig();

    // SPI communication test methods
    bool     verifyRegisterAccess();
    bool     testDriverResponse();
    uint32_t readDriverRegister(uint8_t reg);
    bool     writeDriverRegister(uint8_t reg, uint32_t value);

    // Helper methods
    bool validateConfiguration();
    void initializeStatusStructures();
};