#include "motor_controller.h"
#include <SPI.h>

// Add these definitions at the top of the file
#define TMC5160_DRVSTATUS_OTPW_bm (1UL << 26)  // Over-temperature pre-warning flag
#define TMC5160_DRVSTATUS_OT_bm   (1UL << 25)  // Over-temperature flag

MotorController::MotorController()
    : driver(Config::SPIPins::CS, Config::SPIPins::MOSI, Config::SPIPins::MISO,
             Config::SPIPins::SCK),
      isInitialized(false),
      isConfigured(false) {
    initializeStatusStructures();
}

void MotorController::initializeStatusStructures() {
    // Initialize with safe default values
    driverStatus = {.chopperMode = 0, .enabled = false, .microsteps = 16, .currentSetting = 0};

    motorStatus = {
        .currentPosition = 0, .targetPosition = 0, .currentSpeed = 0, .runMode = 0, .motorLoad = 0};

    thermalStatus = {.currentTemp       = 0,
                     .warningTemp       = 70,
                     .maxTemp           = 85,
                     .driverTemp        = 0,
                     .driverWarningTemp = 80,
                     .driverMaxTemp     = 100,
                     .protectionActive  = false,
                     .protectionLevel   = 0};

    powerStatus = {.supplyVoltage = 0, .motorCurrent = 0, .efficiency = 0};

    errorFlags = {.overtemperature = false,
                  .shortCircuit    = false,
                  .openLoad        = false,
                  .overCurrent     = false,
                  .stallGuard      = false};

    stepperConfig = {.microsteps = 16, .currentScale = 100, .holdCurrent = 500, .runCurrent = 1000};
}

// Status update methods
void MotorController::updateDriverStatus() {
    auto* nonConstThis          = const_cast<MotorController*>(this);
    driverStatus.chopperMode    = nonConstThis->driver.en_pwm_mode() ? 1 : 0;
    driverStatus.enabled        = !digitalRead(Config::MotorPins::EN);
    driverStatus.microsteps     = nonConstThis->driver.microsteps();
    driverStatus.currentSetting = nonConstThis->driver.rms_current();
}

void MotorController::updateMotorStatus() {
    auto* nonConstThis = const_cast<MotorController*>(this);
    // Update motor status from driver registers
    motorStatus.currentPosition = nonConstThis->driver.XACTUAL();
    motorStatus.targetPosition  = nonConstThis->driver.XTARGET();
    motorStatus.currentSpeed = abs(nonConstThis->driver.VACTUAL()) / 8.0f;  // Convert to steps/sec

    // Update run mode based on actual conditions
    if (nonConstThis->driver.VACTUAL() != 0) {
        motorStatus.runMode = 2;  // Velocity mode
    } else if (motorStatus.currentPosition != motorStatus.targetPosition) {
        motorStatus.runMode = 1;  // Position mode
    } else {
        motorStatus.runMode = 0;  // Stopped
    }

    // Calculate motor load based on stallGuard reading
    uint32_t sg_result    = nonConstThis->driver.sg_result();
    motorStatus.motorLoad = 100.0f * (1024 - sg_result) / 1024;
}

void MotorController::updateThermalStatus() {
    try {
        // Read DRV_STATUS register for temperature
        uint32_t drv_status = driver.DRV_STATUS();

        // Update temperature values
        thermalStatus.driverTemp  = ((drv_status >> 24) & 0xFF) * 0.488f;  // Convert to Celsius
        thermalStatus.currentTemp = thermalStatus.driverTemp;              // Motor temp estimation

        // Check temperature flags from DRV_STATUS
        bool otpw = (drv_status & TMC5160_DRVSTATUS_OTPW_bm) != 0;
        bool ot   = (drv_status & TMC5160_DRVSTATUS_OT_bm) != 0;

        // Update protection status
        thermalStatus.protectionActive = otpw || ot;

        // Update error flags
        errorFlags.overtemperature = ot;

        // Calculate protection level (0-100%)
        if (thermalStatus.currentTemp >= thermalStatus.maxTemp) {
            thermalStatus.protectionLevel = 100;
        } else if (thermalStatus.currentTemp >= thermalStatus.warningTemp) {
            float range                   = thermalStatus.maxTemp - thermalStatus.warningTemp;
            float excess                  = thermalStatus.currentTemp - thermalStatus.warningTemp;
            thermalStatus.protectionLevel = static_cast<uint8_t>((excess / range) * 100);
        } else {
            thermalStatus.protectionLevel = 0;
        }
    } catch (...) {
        handleError("Failed to update thermal status");
    }
}

void MotorController::updatePowerStatus() {
    // Read power-related registers
    powerStatus.motorCurrent  = driver.rms_current();
    powerStatus.supplyVoltage = 24.0f;  // Example fixed value, implement actual measurement

    // Calculate efficiency (simplified)
    float nominalPower     = 24.0f * (powerStatus.motorCurrent / 1000.0f);
    float actualPower      = powerStatus.supplyVoltage * (powerStatus.motorCurrent / 1000.0f);
    powerStatus.efficiency = (nominalPower / actualPower) * 100;
}

void MotorController::updateErrorFlags() {
    auto*    nonConstThis = const_cast<MotorController*>(this);
    uint32_t drv_status   = nonConstThis->driver.DRV_STATUS();

    // Fix: Correct bit positions according to TMC5160 datasheet
    errorFlags.overtemperature = (drv_status & (1UL << 26)) != 0;    // OT flag
    errorFlags.shortCircuit    = (drv_status & (1UL << 12)) != 0;    // s2g
    errorFlags.openLoad        = ((drv_status & (3UL << 10)) != 0);  // ola | olb
    errorFlags.stallGuard      = (drv_status & (1UL << 24)) != 0;    // stallGuard
    errorFlags.overCurrent     = false;  // TMC5160 handles overcurrent internally
}

void MotorController::updateStepperConfig() {
    auto* nonConstThis       = const_cast<MotorController*>(this);
    stepperConfig.microsteps = nonConstThis->driver.microsteps();

    // Get IHOLD_IRUN register value for current settings
    uint32_t ihold_irun = nonConstThis->driver.IHOLD_IRUN();
    uint8_t  irun       = (ihold_irun >> 8) & 0x1F;  // IRUN is bits 8-12
    uint8_t  ihold      = ihold_irun & 0x1F;         // IHOLD is bits 0-4

    stepperConfig.currentScale = (irun * 100) / 31;
    stepperConfig.holdCurrent  = (ihold * nonConstThis->driver.rms_current()) / 31;
    stepperConfig.runCurrent   = nonConstThis->driver.rms_current();
}

// Getter methods
MotorController::DriverStatus MotorController::getDriverStatus() const {
    const_cast<MotorController*>(this)->updateDriverStatus();
    return driverStatus;
}

MotorController::MotorStatus MotorController::getMotorStatus() const {
    const_cast<MotorController*>(this)->updateMotorStatus();
    return motorStatus;
}

MotorController::ThermalStatus MotorController::getThermalStatus() const {
    const_cast<MotorController*>(this)->updateThermalStatus();
    return thermalStatus;
}

MotorController::PowerStatus MotorController::getPowerStatus() const {
    const_cast<MotorController*>(this)->updatePowerStatus();
    return powerStatus;
}

MotorController::ErrorFlags MotorController::getErrorFlags() const {
    const_cast<MotorController*>(this)->updateErrorFlags();
    return errorFlags;
}

MotorController::StepperConfig MotorController::getStepperConfig() const {
    const_cast<MotorController*>(this)->updateStepperConfig();
    return stepperConfig;
}

uint16_t MotorController::getMicrostepResolution() const {
    // Using const_cast since driver methods aren't marked const
    auto* nonConstThis = const_cast<MotorController*>(this);
    return nonConstThis->driver.microsteps();
}

uint16_t MotorController::getCurrentSetting() const {
    auto* nonConstThis = const_cast<MotorController*>(this);
    return nonConstThis->driver.rms_current();
}

bool MotorController::initialize() {
    initializePins();
    initializeSPICommunication();
    driver.begin();
    delay(1000);

    // Add SPI connection test
    auto testResult = testConnection();
    if (!testResult.success) {
        handleError(testResult.errorMessage.c_str());
        return false;
    }

    isInitialized = true;
    return true;
}

void MotorController::initializePins() {
    pinMode(Config::MotorPins::STEP, OUTPUT);
    pinMode(Config::MotorPins::DIR, OUTPUT);
    pinMode(Config::MotorPins::EN, OUTPUT);
}

void MotorController::initializeSPICommunication() {
    SPI.begin(Config::SPIPins::SCK, Config::SPIPins::MISO, Config::SPIPins::MOSI);
}

bool MotorController::configure() {
    if (!isInitialized) {
        handleError("Must initialize before configuring");
        return false;
    }

    if (!configureDriverSettings()) {
        handleError("Driver configuration failed");
        return false;
    }

    isConfigured = true;
    return true;
}

bool MotorController::configureDriverSettings() {
    try {
        // First, disable the motor
        disable();
        delay(100);  // Wait for motor to stabilize

        // Configure chopper settings
        const uint32_t CHOPCONF_VALUE = 0x00010153;  // Standard chopper configuration
        driver.CHOPCONF(CHOPCONF_VALUE);
        delay(10);

        // Set current and microsteps with proper scaling
        uint16_t current    = Config::MotorParams::CURRENT_MA;
        uint8_t  microsteps = Config::MotorParams::MICROSTEPS;

        // Set current with proper scaling for TMC5160
        driver.rms_current(current);
        delay(10);

        // Set microsteps
        driver.microsteps(microsteps);
        delay(10);

        // Configure PWM settings
        driver.PWMCONF(0x000504C8);  // PWM configuration for StealthChop
        delay(10);

        // Enable StealthChop
        driver.en_pwm_mode(true);
        driver.GCONF(0x00000004);  // Enable StealthChop
        delay(10);

        // Set power down and threshold settings
        driver.TPOWERDOWN(0x0A);      // ~150ms delay until power down
        driver.TPWMTHRS(0x000001F4);  // Upper velocity for StealthChop
        delay(10);

        // Set initial position mode
        driver.RAMPMODE(1);  // Position mode
        delay(10);

        // Set initial acceleration and velocity
        driver.AMAX(Config::MotorParams::ACCELERATION);
        driver.VMAX(Config::MotorParams::MAX_SPEED);
        delay(10);

        // Set initial position to 0
        driver.XACTUAL(0);
        driver.XTARGET(0);
        delay(10);

        // Verify settings with proper scaling
        uint32_t chopconf_read = driver.CHOPCONF();
        if ((chopconf_read & 0x00010153) != (CHOPCONF_VALUE & 0x00010153)) {
            char errorMsg[100];
            snprintf(errorMsg, sizeof(errorMsg),
                     "CHOPCONF verification failed. Expected: 0x%08X, Got: 0x%08X", CHOPCONF_VALUE,
                     chopconf_read);
            handleError(errorMsg);
            return false;
        }

        // Verify current setting with tolerance
        uint16_t current_read = driver.rms_current();
        if (abs(current_read - current) > 10) {  // Allow 10mA tolerance
            char errorMsg[100];
            snprintf(errorMsg, sizeof(errorMsg),
                     "Current setting verification failed. Expected: %d mA, Got: %d mA", current,
                     current_read);
            handleError(errorMsg);
            return false;
        }

        // Verify microsteps
        if (driver.microsteps() != microsteps) {
            char errorMsg[100];
            snprintf(errorMsg, sizeof(errorMsg),
                     "Microsteps verification failed. Expected: %d, Got: %d", microsteps,
                     driver.microsteps());
            handleError(errorMsg);
            return false;
        }

        // Verify RAMPMODE
        if (driver.RAMPMODE() != 1) {
            handleError("RAMPMODE verification failed");
            return false;
        }

        // Debug messages
        Serial.println("\nDriver configuration completed:");
        Serial.print("Current: ");
        Serial.println(driver.rms_current());
        Serial.print("Microsteps: ");
        Serial.println(driver.microsteps());
        Serial.print("Mode: ");
        Serial.println(driver.RAMPMODE());
        Serial.print("Position: ");
        Serial.println(driver.XACTUAL());
        Serial.print("Acceleration: ");
        Serial.println(driver.AMAX());
        Serial.print("Max speed: ");
        Serial.println(driver.VMAX());
        Serial.print("CHOPCONF: 0x");
        Serial.println(driver.CHOPCONF(), HEX);

        return true;
    } catch (...) {
        handleError("Failed to configure driver settings");
        return false;
    }
}

void MotorController::handleError(const char* errorMsg) {
    Serial.println(errorMsg);
    // Additional error handling logic
}

bool MotorController::enable() {
    if (!isReady())
        return false;
    digitalWrite(Config::MotorPins::EN, LOW);
    return true;
}

bool MotorController::disable() {
    digitalWrite(Config::MotorPins::EN, HIGH);
    return true;
}

float MotorController::getCurrentTemperature() const {
    // Implementation for temperature monitoring
    return 0.0f;
}

uint32_t MotorController::getCurrentSpeed() const {
    // Implementation for speed monitoring
    return 0;
}

bool MotorController::isEnabled() const {
    return digitalRead(Config::MotorPins::EN) == LOW;
}

bool MotorController::testVersionMatch() {
    const uint8_t TMC5160_VERSION = 0x30;  // Expected TMC5160 version
    uint32_t      version         = 0;

    try {
        // Read version with retries
        for (int i = 0; i < 3; i++) {
            version = driver.version();
            if ((version & 0xFF) == TMC5160_VERSION) {  // Check only the last byte
                return true;
            }
            delayMicroseconds(100);
        }

        char errorMsg[50];
        snprintf(errorMsg, sizeof(errorMsg), "Version mismatch. Expected: 0x%02X, Got: 0x%02X",
                 TMC5160_VERSION, version & 0xFF);
        handleError(errorMsg);
        return false;
    } catch (...) {
        handleError("Failed to read driver version");
        return false;
    }
}

bool MotorController::testRegisterAccess() {
    const uint32_t TEST_VALUE = 0x00010153;  // Test value for CHOPCONF register

    try {
        // Save current CHOPCONF value
        uint32_t originalValue = driver.CHOPCONF();

        // Write test value
        driver.CHOPCONF(TEST_VALUE);
        delayMicroseconds(100);

        // Read back value
        uint32_t readValue = driver.CHOPCONF();

        // Restore original value
        driver.CHOPCONF(originalValue);

        if (readValue != TEST_VALUE) {
            char errorMsg[100];
            snprintf(errorMsg, sizeof(errorMsg),
                     "Register access failed. Written: 0x%08X, Read: 0x%08X", TEST_VALUE,
                     readValue);
            handleError(errorMsg);
            return false;
        }

        return true;
    } catch (...) {
        handleError("Failed to test register access");
        return false;
    }
}

bool MotorController::testErrorStatus() {
    try {
        uint32_t drv_status = driver.DRV_STATUS();

        // Check for critical error bits
        if (drv_status & (1UL << 26)) {  // OT flag
            handleError("Over-temperature error");
            return false;
        }

        if (drv_status & (1UL << 12)) {  // Short circuit
            handleError("Short circuit detected");
            return false;
        }

        // Check for warning conditions
        if (drv_status & TMC5160_DRVSTATUS_OTPW_bm) {
            handleError("Temperature pre-warning active");
            return false;
        }

        return true;
    } catch (...) {
        handleError("Failed to read driver status");
        return false;
    }
}

bool MotorController::testSPICommunication() {
    const uint32_t MAX_TEST_TIME = 10000;  // 10ms timeout
    const uint32_t startTime     = micros();

    // Test version match
    if (!testVersionMatch()) {
        return false;
    }

    // Check timeout
    if (micros() - startTime > MAX_TEST_TIME) {
        handleError("SPI test timeout");
        return false;
    }

    // Test register access
    if (!testRegisterAccess()) {
        return false;
    }

    // Check timeout
    if (micros() - startTime > MAX_TEST_TIME) {
        handleError("SPI test timeout");
        return false;
    }

    // Test error status
    if (!testErrorStatus()) {
        return false;
    }

    return true;
}

bool MotorController::verifyRegisterAccess() {
    try {
        // Test register write/read functionality using CHOPCONF register
        const uint32_t TEST_VALUE = 0x10410153;  // Standard CHOPCONF value

        // Save original value
        uint32_t originalValue = driver.CHOPCONF();

        // Write test value
        driver.CHOPCONF(TEST_VALUE);
        delayMicroseconds(100);  // Short delay for register update

        // Read back the value
        uint32_t readValue = driver.CHOPCONF();

        // Restore original value
        driver.CHOPCONF(originalValue);

        if (readValue != TEST_VALUE) {
            char errorMsg[100];
            snprintf(errorMsg, sizeof(errorMsg),
                     "Register verification failed. Written: 0x%08X, Read: 0x%08X", TEST_VALUE,
                     readValue);
            handleError(errorMsg);
            return false;
        }

        return true;
    } catch (...) {
        handleError("Exception during register verification");
        return false;
    }
}

bool MotorController::testDriverResponse() {
    try {
        auto* nonConstThis = const_cast<MotorController*>(this);

        // Test GCONF register
        nonConstThis->driver.GCONF(0x00000004);
        delay(10);

        uint32_t gconf = nonConstThis->driver.GCONF();
        if ((gconf & 0x00000004) != 0x00000004) {
            handleError("Driver response test failed - GCONF mismatch");
            return false;
        }

        // Test CHOPCONF register
        nonConstThis->driver.CHOPCONF(0x10410153);
        delay(10);

        uint32_t chopconf = nonConstThis->driver.CHOPCONF();
        if (chopconf != 0x10410153) {
            handleError("Driver response test failed - CHOPCONF mismatch");
            return false;
        }

        // Test PWMCONF register
        nonConstThis->driver.PWMCONF(0x000504C8);
        delay(10);

        uint32_t pwmconf = nonConstThis->driver.PWMCONF();
        if (pwmconf != 0x000504C8) {
            handleError("Driver response test failed - PWMCONF mismatch");
            return false;
        }

        return true;
    } catch (...) {
        handleError("Exception during driver response test");
        return false;
    }
}

uint32_t MotorController::readDriverRegister(uint8_t reg) {
    uint32_t response = 0;

    digitalWrite(Config::SPIPins::CS, LOW);
    delayMicroseconds(10);

    SPI.transfer(reg & 0x7F);  // Send register address for reading

    // Read 4 bytes
    for (int i = 0; i < 4; i++) {
        response <<= 8;
        response |= SPI.transfer(0x00);
    }

    digitalWrite(Config::SPIPins::CS, HIGH);
    delayMicroseconds(10);

    return response;
}

bool MotorController::writeDriverRegister(uint8_t reg, uint32_t value) {
    digitalWrite(Config::SPIPins::CS, LOW);
    delayMicroseconds(10);

    SPI.transfer(reg | 0x80);  // Send register address for writing

    // Write 4 bytes
    for (int i = 3; i >= 0; i--) {
        SPI.transfer((value >> (8 * i)) & 0xFF);
    }

    digitalWrite(Config::SPIPins::CS, HIGH);
    delayMicroseconds(10);

    return true;
}

MotorController::SPITestResult MotorController::testConnection() {
    SPITestResult result = {false, 0, ""};

    // Step 1: Test basic SPI communication
    if (!testSPICommunication()) {
        result.errorMessage = "SPI communication test failed";
        return result;
    }

    // Step 2: Verify register access
    if (!verifyRegisterAccess()) {
        result.errorMessage = "Register access verification failed";
        return result;
    }

    // Step 3: Test driver response
    if (!testDriverResponse()) {
        result.errorMessage = "Driver response test failed";
        return result;
    }

    // All tests passed
    result.success      = true;
    result.version      = driver.version();
    result.errorMessage = "Connection test passed";
    return result;
}

bool MotorController::isSPIConnected() const {
    const uint8_t TMC5160_VERSION = 0x30;
    // Use non-const method through const_cast since version() isn't marked const
    auto* nonConstThis = const_cast<MotorController*>(this);
    return nonConstThis->driver.version() == TMC5160_VERSION;
}

bool MotorController::emergencyStop() {
    if (!isReady())
        return false;

    try {
        // Stop immediately using velocity mode
        driver.VMAX(0);                    // Set maximum velocity to 0
        driver.AMAX(65535);                // Set maximum acceleration to max
        driver.XTARGET(driver.XACTUAL());  // Set target to current position

        // Optional: Disable motor after stop
        disable();

        return true;
    } catch (...) {
        return false;
    }
}

float MotorController::getTemperature() {
    try {
        // Read temperature from DRV_STATUS register
        uint32_t drv_status = driver.DRV_STATUS();

        // Extract temperature bits (bits 24-31)
        uint8_t temp = (drv_status >> 24) & 0xFF;

        // Debug print raw temperature value
        Serial.print("Raw temperature value (0-255): ");
        Serial.println(temp);
        Serial.print("Raw temperature value (hex): 0x");
        Serial.println(temp, HEX);

        // Convert to Celsius using the correct scaling factor
        // TMC5160 datasheet: Temperature = (temp * 0.0046875) + 25
        float temperature = (temp * 0.0046875f) + 25.0f;

        // Update thermal status
        thermalStatus.currentTemp = temperature;

        return temperature;
    } catch (const std::exception& e) {
        handleError("Failed to read temperature");
        return 0.0f;
    }
}

bool MotorController::overTemperaturePreWarning() {
    try {
        uint32_t drv_status = driver.DRV_STATUS();
        return (drv_status & (1UL << 25)) != 0;  // Check OT bit
    } catch (...) {
        handleError("Failed to read temperature warning status");
        return false;
    }
}

bool MotorController::overTemperatureFlag() {
    try {
        uint32_t drv_status = driver.DRV_STATUS();
        return (drv_status & (1UL << 26)) != 0;  // Check OT bit
    } catch (...) {
        handleError("Failed to read temperature flag");
        return false;
    }
}

bool MotorController::setSpeed(uint32_t speed) {
    if (!isReady())
        return false;

    try {
        driver.VMAX(speed);  // Set maximum velocity
        return true;
    } catch (...) {
        return false;
    }
}

bool MotorController::setMotorCurrent(uint16_t current) {
    if (!isReady())
        return false;

    try {
        // Set the RMS current
        driver.rms_current(current);

        // Update the stepper configuration
        stepperConfig.runCurrent = current;

        // Set hold current to 50% of run current
        uint8_t ihold = (current * 16) / 31;  // Scale for hold current (approximately 50%)
        uint8_t irun  = (current * 31) / 31;  // Full current for run

        // Update IHOLD_IRUN register
        // IHOLD[4:0] | IRUN[4:0] | IHOLDDELAY[3:0]
        driver.IHOLD_IRUN((irun << 8) | (ihold));

        return true;
    } catch (...) {
        handleError("Failed to set motor current");
        return false;
    }
}

bool MotorController::setDirection(bool clockwise) {
    if (!isReady())
        return false;

    try {
        // Set direction pin (HIGH for clockwise, LOW for counter-clockwise)
        digitalWrite(Config::MotorPins::DIR, clockwise ? HIGH : LOW);
        delayMicroseconds(10);  // Small delay to ensure pin state is stable

        // Debug message
        Serial.print("Direction pin set to: ");
        Serial.println(clockwise ? "HIGH" : "LOW");

        return true;
    } catch (...) {
        handleError("Failed to set direction");
        return false;
    }
}

bool MotorController::moveToPosition(int32_t targetPosition) {
    if (!isReady()) {
        handleError("Motor not ready");
        return false;
    }

    try {
        // Enable the motor if not already enabled
        if (!isEnabled()) {
            if (!enable()) {
                handleError("Failed to enable motor");
                return false;
            }
            delay(100);  // Wait longer for motor to stabilize
        }

        // Get current position
        int32_t currentPos = driver.XACTUAL();

        // Calculate direction based on target position
        bool clockwise = targetPosition > currentPos;

        // Set direction pin
        digitalWrite(Config::MotorPins::DIR, clockwise ? HIGH : LOW);
        delay(50);  // Wait longer for direction pin to stabilize

        // Set the motor to position mode
        driver.RAMPMODE(1);  // Position mode
        delay(50);           // Wait longer for mode change

        // Set acceleration and velocity
        driver.AMAX(Config::MotorParams::ACCELERATION);
        driver.VMAX(Config::MotorParams::MAX_SPEED);
        delay(50);  // Wait longer for settings to take effect

        // Set target position and start movement
        driver.XTARGET(targetPosition);

        // Debug messages
        Serial.println("\nStarting movement:");
        Serial.print("Current position: ");
        Serial.println(currentPos);
        Serial.print("Target position: ");
        Serial.println(targetPosition);
        Serial.print("Direction: ");
        Serial.println(clockwise ? "clockwise" : "counter-clockwise");
        Serial.print("Current speed: ");
        Serial.println(driver.VACTUAL());
        Serial.print("Acceleration: ");
        Serial.println(driver.AMAX());
        Serial.print("Max speed: ");
        Serial.println(driver.VMAX());
        Serial.print("Motor enabled: ");
        Serial.println(isEnabled());
        Serial.print("Driver status: 0x");
        Serial.println(driver.DRV_STATUS(), HEX);

        // Wait for movement to complete with timeout
        const uint32_t startTime        = millis();
        const uint32_t timeout          = 5000;  // 5 second timeout
        bool           movementComplete = false;
        int32_t        lastPosition     = currentPos;
        uint32_t       noChangeCount    = 0;

        while (!movementComplete && (millis() - startTime < timeout)) {
            int32_t currentPosition = driver.XACTUAL();

            // Check if we've reached the target position
            if (currentPosition == targetPosition) {
                movementComplete = true;
                break;
            }

            // Check if position has changed
            if (currentPosition == lastPosition) {
                noChangeCount++;
                if (noChangeCount > 10) {  // If position hasn't changed for 1 second
                    handleError("Motor stuck - position not changing");
                    emergencyStop();
                    return false;
                }
            } else {
                noChangeCount = 0;
                lastPosition  = currentPosition;
            }

            // Check for errors
            uint32_t drv_status = driver.DRV_STATUS();
            if (drv_status & (1UL << 12)) {  // Short circuit
                handleError("Short circuit detected");
                emergencyStop();
                return false;
            }
            if (drv_status & (1UL << 10)) {  // Open load
                handleError("Open load detected");
                emergencyStop();
                return false;
            }

            // Update status
            Serial.print("Current position: ");
            Serial.println(currentPosition);
            Serial.print("Current speed: ");
            Serial.println(driver.VACTUAL());
            Serial.print("Driver status: 0x");
            Serial.println(drv_status, HEX);

            delay(100);  // Check every 100ms
        }

        if (!movementComplete) {
            handleError("Movement timeout");
            emergencyStop();
            return false;
        }

        // Movement completed successfully
        Serial.println("\nMovement completed successfully:");
        Serial.print("Final position: ");
        Serial.println(driver.XACTUAL());
        Serial.print("Final speed: ");
        Serial.println(driver.VACTUAL());

        return true;
    } catch (...) {
        handleError("Failed to move to position");
        emergencyStop();
        return false;
    }
}

bool MotorController::autoTune() {
    if (!isReady()) {
        handleError("Motor not ready for auto-tuning");
        return false;
    }

    try {
        Serial.println("\nStarting Auto-Tuning Process...");

        // Disable motor first
        disable();
        delay(100);

        // Enable motor
        enable();
        delay(100);

        // Set initial parameters for tuning
        driver.RAMPMODE(1);           // Position mode
        driver.AMAX(1000);            // Start with low acceleration
        driver.VMAX(1000);            // Start with low velocity
        driver.TPWMTHRS(0);           // Disable StealthChop for tuning
        driver.CHOPCONF(0x00010153);  // Standard chopper configuration
        delay(100);

        // Perform tuning sequence
        Serial.println("Tuning sequence started...");

        // Step 1: Find optimal current
        Serial.println("Step 1: Finding optimal current...");
        uint16_t optimalCurrent = findOptimalCurrent();
        driver.rms_current(optimalCurrent);
        Serial.print("Optimal current found: ");
        Serial.println(optimalCurrent);

        // Step 2: Find optimal microsteps
        Serial.println("Step 2: Finding optimal microsteps...");
        uint16_t optimalMicrosteps = findOptimalMicrosteps();
        driver.microsteps(optimalMicrosteps);
        Serial.print("Optimal microsteps found: ");
        Serial.println(optimalMicrosteps);

        // Step 3: Find optimal acceleration
        Serial.println("Step 3: Finding optimal acceleration...");
        uint32_t optimalAccel = findOptimalAcceleration();
        driver.AMAX(optimalAccel);
        Serial.print("Optimal acceleration found: ");
        Serial.println(optimalAccel);

        // Step 4: Find optimal velocity
        Serial.println("Step 4: Finding optimal velocity...");
        uint32_t optimalVelocity = findOptimalVelocity();
        driver.VMAX(optimalVelocity);
        Serial.print("Optimal velocity found: ");
        Serial.println(optimalVelocity);

        // Step 5: Fine-tune StealthChop
        Serial.println("Step 5: Fine-tuning StealthChop...");
        tuneStealthChop();

        Serial.println("\nAuto-tuning completed successfully!");
        return true;

    } catch (...) {
        handleError("Auto-tuning failed");
        return false;
    }
}

uint16_t MotorController::findOptimalCurrent() {
    uint16_t current        = 100;  // Start with low current
    uint16_t bestCurrent    = current;
    float    bestSmoothness = 0;

    // Test different current values
    for (current = 100; current <= 2000; current += 100) {
        driver.rms_current(current);
        delay(100);

        // Move motor and measure smoothness
        float smoothness = measureMovementSmoothness();

        if (smoothness > bestSmoothness) {
            bestSmoothness = smoothness;
            bestCurrent    = current;
        }
    }

    return bestCurrent;
}

uint16_t MotorController::findOptimalMicrosteps() {
    uint16_t microsteps     = 16;
    uint16_t bestMicrosteps = microsteps;
    float    bestSmoothness = 0;

    // Test different microstep values
    uint16_t testValues[] = {16, 32, 64, 128, 256};
    for (uint16_t ms : testValues) {
        driver.microsteps(ms);
        delay(100);

        // Move motor and measure smoothness
        float smoothness = measureMovementSmoothness();

        if (smoothness > bestSmoothness) {
            bestSmoothness = smoothness;
            bestMicrosteps = ms;
        }
    }

    return bestMicrosteps;
}

uint32_t MotorController::findOptimalAcceleration() {
    uint32_t accel          = 1000;
    uint32_t bestAccel      = accel;
    float    bestSmoothness = 0;

    // Test different acceleration values
    for (accel = 1000; accel <= 10000; accel += 1000) {
        driver.AMAX(accel);
        delay(100);

        // Move motor and measure smoothness
        float smoothness = measureMovementSmoothness();

        if (smoothness > bestSmoothness) {
            bestSmoothness = smoothness;
            bestAccel      = accel;
        }
    }

    return bestAccel;
}

uint32_t MotorController::findOptimalVelocity() {
    uint32_t velocity       = 1000;
    uint32_t bestVelocity   = velocity;
    float    bestSmoothness = 0;

    // Test different velocity values
    for (velocity = 1000; velocity <= 10000; velocity += 1000) {
        driver.VMAX(velocity);
        delay(100);

        // Move motor and measure smoothness
        float smoothness = measureMovementSmoothness();

        if (smoothness > bestSmoothness) {
            bestSmoothness = smoothness;
            bestVelocity   = velocity;
        }
    }

    return bestVelocity;
}

void MotorController::tuneStealthChop() {
    // Enable StealthChop
    driver.en_pwm_mode(true);
    driver.GCONF(0x00000004);
    delay(100);

    // Configure PWM settings for smooth operation
    driver.PWMCONF(0x000504C8);
    delay(100);

    // Set power down and threshold settings
    driver.TPOWERDOWN(0x0A);
    driver.TPWMTHRS(0x000001F4);
    delay(100);
}

float MotorController::measureMovementSmoothness() {
    float smoothness = 0;

    // Move motor a short distance
    int32_t startPos = driver.XACTUAL();
    driver.XTARGET(startPos + 1000);

    // Monitor movement
    uint32_t startTime     = millis();
    int32_t  lastPos       = startPos;
    uint32_t samples       = 0;
    float    totalVariance = 0;

    while (millis() - startTime < 1000) {  // Monitor for 1 second
        int32_t currentPos = driver.XACTUAL();
        int32_t velocity   = currentPos - lastPos;

        // Calculate variance in velocity
        totalVariance += velocity * velocity;
        samples++;

        lastPos = currentPos;
        delay(10);
    }

    // Calculate smoothness (lower variance = smoother movement)
    smoothness = 1.0f / (1.0f + (totalVariance / samples));

    return smoothness;
}