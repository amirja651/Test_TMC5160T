#include "MotorController.h"
#include "config.h"

// Constructor initializes motor driver and sets default parameters
MotorController::MotorController(uint8_t csPin, uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                                 uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin)
    : driver(csPin, mosiPin, misoPin, sckPin),
      isMoving(false),
      direction(true),
      stepDelay(Config::MotorController::STEP_DELAY),
      lastStepTime(0),
      stepCounter(0),
      csPin(csPin),
      stepPin(stepPin),
      dirPin(dirPin),
      enPin(enPin),
      mosiPin(mosiPin),
      misoPin(misoPin),
      sckPin(sckPin),
      runCurrent(Config::MotorSpecs::Operation::RUN_CURRENT),     // Default 1000mA
      holdCurrent(Config::MotorSpecs::Operation::HOLD_CURRENT),   // Default 500mA
      speed(Config::MotorSpecs::Operation::SPEED),                // Default 1000 steps/sec
      acceleration(Config::MotorSpecs::Operation::ACCELERATION),  // Default 1000 steps/sec²
      lastTempPrintTime(0),
      lastTemperature(0) {}

// Initialize motor controller and driver
void MotorController::begin() {
    setupPins();
    delay(100);  // Wait for power to stabilize
    disableSPI();

    // Reset driver state
    disableDriver();
    delay(100);
    enableDriver();
    delay(100);

    configureDriver();
}

// Configure GPIO pins for motor control
void MotorController::setupPins() {
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    pinMode(csPin, OUTPUT);

    disableDriver();
    delay(200);
    enableDriver();
}

// Configure TMC5160 driver parameters
void MotorController::configureDriver() {
    driver.begin();
    delay(200);

    driver.GCONF(0x0);
    delay(100);

    // Set initial currents
    driver.rms_current(runCurrent);
    driver.ihold(holdCurrent);
    delay(100);

    driver.microsteps(Config::TMC5160T_Driver::MICROSTEPS);
    delay(100);

    // Configure CoolStep and basic driver settings
    driver.TPWMTHRS(0);  // Enable StealthChop mode by default
    driver.TCOOLTHRS(Config::TMC5160T_Driver::TCOOLTHRS);
    driver.TPOWERDOWN(Config::TMC5160T_Driver::TPOWERDOWN);

    // Set driver timing parameters
    driver.toff(Config::TMC5160T_Driver::TOFF);
    driver.blank_time(Config::TMC5160T_Driver::BLANK_TIME);
    driver.iholddelay(Config::TMC5160T_Driver::IHOLDDELAY);
    delay(100);

    enableDriver();
    delay(200);
}

// Handle power loss by reinitializing driver
void MotorController::handlePowerLoss() {
    // Disable motor
    disableDriver();
    delay(100);

    // Reset SPI communication
    disableSPI();
    delay(100);
    enableSPI();
    delay(100);

    // Reconfigure driver
    configureDriver();
}

// Check driver status and reinitialize if needed
bool MotorController::checkAndReinitializeDriver() {
    uint32_t status = driver.DRV_STATUS();
    if (status == 0 || status == Config::MotorController::INVALID_STATUS) {
        handlePowerLoss();
        return true;
    }
    return false;
}

// Start motor movement in forward direction
void MotorController::moveForward() {
    uint32_t status = driver.DRV_STATUS();
    if (status & 0x00000001) {  // Check for drive voltage error (bit 0)
        Serial.println("ERROR: Drive voltage error detected. Movement command ignored.");
        Serial.println("Please check power supply and connections.");
        return;
    }

    isMoving  = true;
    direction = true;
    digitalWrite(dirPin, direction ? HIGH : LOW);
    Serial.println("Moving Forward");
}

// Start motor movement in reverse direction
void MotorController::moveReverse() {
    uint32_t status = driver.DRV_STATUS();
    if (status & 0x00000001) {  // Check for drive voltage error (bit 0)
        Serial.println("ERROR: Drive voltage error detected. Movement command ignored.");
        Serial.println("Please check power supply and connections.");
        return;
    }

    isMoving  = true;
    direction = false;
    digitalWrite(dirPin, direction ? HIGH : LOW);
    Serial.println("Moving Reverse");
}

// Stop motor movement
void MotorController::stop() {
    isMoving = false;
}

// Execute a single step
void MotorController::step() {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    lastStepTime = micros();

    if (++stepCounter >= Config::MotorController::STATUS_PRINT_INTERVAL) {
        stepCounter = 0;
    }
}

// Main update loop for motor control
void MotorController::update() {
    if (isMoving) {
        if (micros() - lastStepTime >= (1000000 / speed)) {
            step();
        }

        // Check for stall condition
        checkStall();

        // Monitor temperature and adjust current if needed
        int temp = getTemperature();
        if (temp > Config::TMC5160T_Driver::TEMP_WARNING_THRESHOLD) {
            Serial.print("WARNING: High temperature detected: ");
            Serial.println(temp);
            // Reduce current by 20% if temperature is high
            uint16_t reducedCurrent = runCurrent * 0.8;
            driver.rms_current(reducedCurrent);
            Serial.print("Reduced current to: ");
            Serial.println(reducedCurrent);
        }

        // Print temperature at configured interval
        if (millis() - lastTempPrintTime >= Config::TMC5160T_Driver::TEMP_PRINT_INTERVAL) {
            printTemperature();
            lastTempPrintTime = millis();
        }
    }
}

uint32_t MotorController::getDriverStatus() {
    return driver.DRV_STATUS();
}

void MotorController::increaseRunCurrent() {
    if (runCurrent < Config::MotorController::MAX_RUN_CURRENT) {
        runCurrent += Config::MotorController::CURRENT_STEP;
        driver.rms_current(runCurrent);
        Serial.print("Run current increased to: ");
        Serial.print(runCurrent);
        Serial.println("mA (Max: 1000mA)");
    } else {
        Serial.println("Run current at maximum (1000mA)");
    }
}

void MotorController::decreaseRunCurrent() {
    if (runCurrent > Config::MotorController::MIN_CURRENT) {
        runCurrent -= Config::MotorController::CURRENT_STEP;
        driver.rms_current(runCurrent);
        Serial.print("Run current decreased to: ");
        Serial.print(runCurrent);
        Serial.println("mA (Min: 100mA)");
    } else {
        Serial.println("Run current at minimum (100mA)");
    }
}

void MotorController::increaseHoldCurrent() {
    if (holdCurrent < Config::MotorController::MAX_HOLD_CURRENT) {
        holdCurrent += Config::MotorController::CURRENT_STEP;
        driver.ihold(holdCurrent);
        Serial.print("Hold current increased to: ");
        Serial.print(holdCurrent);
        Serial.println("mA (Max: 500mA)");
    } else {
        Serial.println("Hold current at maximum (500mA)");
    }
}

void MotorController::decreaseHoldCurrent() {
    if (holdCurrent > Config::MotorController::MIN_CURRENT) {
        holdCurrent -= Config::MotorController::CURRENT_STEP;
        driver.ihold(holdCurrent);
        Serial.print("Hold current decreased to: ");
        Serial.print(holdCurrent);
        Serial.println("mA (Min: 100mA)");
    } else {
        Serial.println("Hold current at minimum (100mA)");
    }
}

uint16_t MotorController::getRunCurrent() const {
    return runCurrent;
}

uint16_t MotorController::getHoldCurrent() const {
    return holdCurrent;
}

void MotorController::increaseSpeed() {
    if (speed < Config::MotorController::MAX_SPEED) {
        speed += Config::MotorController::SPEED_STEP;
        Serial.print("Speed increased to: ");
        Serial.print(speed);
        Serial.println(" steps/sec");
    } else {
        Serial.println("Speed at maximum (10000 steps/sec)");
    }
}

void MotorController::decreaseSpeed() {
    if (speed > Config::MotorController::MIN_SPEED) {
        speed -= Config::MotorController::SPEED_STEP;
        Serial.print("Speed decreased to: ");
        Serial.print(speed);
        Serial.println(" steps/sec");
    } else {
        Serial.println("Speed at minimum (100 steps/sec)");
    }
}

void MotorController::increaseAcceleration() {
    if (acceleration < Config::MotorController::MAX_ACCEL) {
        acceleration += Config::MotorController::ACCEL_STEP;
        driver.AMAX(acceleration);
        Serial.print("Acceleration increased to: ");
        Serial.print(acceleration);
        Serial.println(" steps/sec²");
    } else {
        Serial.println("Acceleration at maximum (10000 steps/sec²)");
    }
}

void MotorController::decreaseAcceleration() {
    if (acceleration > Config::MotorController::MIN_ACCEL) {
        acceleration -= Config::MotorController::ACCEL_STEP;
        driver.AMAX(acceleration);
        Serial.print("Acceleration decreased to: ");
        Serial.print(acceleration);
        Serial.println(" steps/sec²");
    } else {
        Serial.println("Acceleration at minimum (100 steps/sec²)");
    }
}

uint16_t MotorController::getSpeed() const {
    return speed;
}

uint16_t MotorController::getAcceleration() const {
    return acceleration;
}

void MotorController::printStatusRegister(uint32_t status) {
    Serial.println("\nDriver Status Register:");
    Serial.print("Raw Status: 0x");
    Serial.println(status, HEX);
    printErrorFlags(status);
    printStallGuardStatus(status);
    printDriverState(status);
}

void MotorController::printErrorFlags(uint32_t status) {
    Serial.println("\nError Flags:");
    Serial.print("  Over Temperature: ");
    Serial.println((status & 0x00000001) ? "Yes" : "No");
    Serial.print("  Short to Ground A: ");
    Serial.println((status & 0x00000002) ? "Yes" : "No");
    Serial.print("  Short to Ground B: ");
    Serial.println((status & 0x00000004) ? "Yes" : "No");
    Serial.print("  Open Load A: ");
    Serial.println((status & 0x00000008) ? "Yes" : "No");
    Serial.print("  Open Load B: ");
    Serial.println((status & 0x00000010) ? "Yes" : "No");
}

void MotorController::printStallGuardStatus(uint32_t status) {
    Serial.println("\nStallGuard Status:");
    Serial.print("  StallGuard Value: ");
    Serial.println((status >> 10) & 0x3FF);
    Serial.print("  Stall Detected: ");
    Serial.println((status & 0x00000200) ? "Yes" : "No");
}

void MotorController::printDriverState(uint32_t status) {
    Serial.println("\nDriver State:");
    Serial.print("  Standstill: ");
    Serial.println((status & 0x00000400) ? "Yes" : "No");
    Serial.print("  Velocity Reached: ");
    Serial.println((status & 0x00000800) ? "Yes" : "No");
    Serial.print("  Position Reached: ");
    Serial.println((status & 0x00001000) ? "Yes" : "No");
}

void MotorController::printDriverStatus() {
    uint32_t status = driver.DRV_STATUS();

    Serial.println("\n=== TMC5160 Driver Status Report ===");
    Serial.println("=====================================");

    // Basic Status
    Serial.println("\n1. Basic Status:");
    Serial.println("----------------");
    Serial.print("  Raw Status Register: 0x");
    Serial.println(status, HEX);

    // Error Status
    Serial.println("\n2. Error Status:");
    Serial.println("----------------");
    Serial.print("  Over Temperature: ");
    Serial.println((status & 0x00000001) ? "WARNING - Temperature too high!" : "OK");
    Serial.print("  Short to Ground A: ");
    Serial.println((status & 0x00000002) ? "ERROR - Phase A shorted!" : "OK");
    Serial.print("  Short to Ground B: ");
    Serial.println((status & 0x00000004) ? "ERROR - Phase B shorted!" : "OK");
    Serial.print("  Open Load A: ");
    Serial.println((status & 0x00000008) ? "ERROR - Phase A open!" : "OK");
    Serial.print("  Open Load B: ");
    Serial.println((status & 0x00000010) ? "ERROR - Phase B open!" : "OK");

    // StallGuard Status
    Serial.println("\n3. StallGuard Status:");
    Serial.println("---------------------");
    Serial.print("  StallGuard Value: ");
    Serial.println((status >> 10) & 0x3FF);
    Serial.print("  Stall Detected: ");
    Serial.println((status & 0x00000200) ? "WARNING - Motor stalled!" : "OK");

    // Driver State
    Serial.println("\n4. Driver State:");
    Serial.println("----------------");
    Serial.print("  Standstill: ");
    Serial.println((status & 0x00000400) ? "Yes" : "No");
    Serial.print("  Velocity Reached: ");
    Serial.println((status & 0x00000800) ? "Yes" : "No");
    Serial.print("  Position Reached: ");
    Serial.println((status & 0x00001000) ? "Yes" : "No");

    // Temperature
    Serial.println("\n5. Temperature Status:");
    Serial.println("----------------------");
    int temp = getTemperature();
    Serial.print("  Current Temperature: ");
    Serial.print(temp);
    Serial.println("°C");
    if (temp > 60) {
        Serial.println("  WARNING: Temperature above 60°C!");
    } else if (temp > 45) {
        Serial.println("  NOTE: Temperature is getting high");
    }

    // Operating Mode
    Serial.println("\n6. Operating Mode:");
    Serial.println("------------------");
    Serial.print("  StealthChop Mode: ");
    Serial.println((driver.TPWMTHRS() == 0) ? "Enabled" : "Disabled");
    Serial.print("  SpreadCycle Mode: ");
    Serial.println((driver.TPWMTHRS() > 0) ? "Enabled" : "Disabled");

    // Current Settings
    Serial.println("\n7. Current Settings:");
    Serial.println("-------------------");
    Serial.print("  Run Current: ");
    Serial.print(runCurrent);
    Serial.println("mA");
    Serial.print("  Hold Current: ");
    Serial.print(holdCurrent);
    Serial.println("mA");

    // Motion Settings
    Serial.println("\n8. Motion Settings:");
    Serial.println("-------------------");
    Serial.print("  Speed: ");
    Serial.print(speed);
    Serial.println(" steps/sec");
    Serial.print("  Acceleration: ");
    Serial.print(acceleration);
    Serial.println(" steps/sec²");

    Serial.println("\n=====================================");
}

void MotorController::printDriverConfig() {
    Serial.println("\nDriver Configuration:");
    Serial.println("-------------------");
    Serial.print("  Run Current: ");
    Serial.print(runCurrent);
    Serial.println("mA");
    Serial.print("  Hold Current: ");
    Serial.print(holdCurrent);
    Serial.println("mA");
    Serial.print("  Microsteps: ");
    Serial.println(16);
    Serial.print("  Speed: ");
    Serial.print(speed);
    Serial.println(" steps/sec");
    Serial.print("  Acceleration: ");
    Serial.print(acceleration);
    Serial.println(" steps/sec²");

    Serial.println("\nDriver Parameters:");
    Serial.println("------------------");
    Serial.print("  GCONF (Global Config): 0x");
    Serial.println(driver.GCONF(), HEX);
    Serial.print("  TPOWERDOWN (Power Down Time): ");
    Serial.print(driver.TPOWERDOWN());
    Serial.println(" tclk");
    Serial.print("  TSTEP (Current Step Timing): ");
    Serial.print(driver.TSTEP());
    Serial.println(" tclk");
    Serial.print("  TPWMTHRS (StealthChop Threshold): ");
    Serial.print(driver.TPWMTHRS());
    Serial.println(" tclk");
    Serial.print("  THIGH (Step Pulse High Time): ");
    Serial.print(driver.THIGH());
    Serial.println(" tclk");
    Serial.print("  XDIRECT (Direct Coil Control): 0x");
    Serial.println(driver.XDIRECT(), HEX);
}

int MotorController::getTemperature() {
    uint32_t status  = driver.DRV_STATUS();
    int      rawTemp = (status >> 16) & 0xFF;  // Temperature is in bits 16-23
    return (rawTemp - 1) * 1.5;                // Convert to actual temperature in Celsius
}

void MotorController::printTemperature() {
    int temp = getTemperature();
    if (temp != lastTemperature) {
        Serial.print("Temperature: ");
        Serial.print(temp);
        Serial.println("°C");
        lastTemperature = temp;
    }
}

// Check for motor stall condition
void MotorController::checkStall() {
    uint32_t status = driver.DRV_STATUS();
    if (status & Config::TMC5160T_Driver::STALL_BIT_MASK) {
        Serial.println("WARNING: Stall detected!");
        stop();  // Stop motor on stall
        printStallGuardStatus(status);
    }
}

// Toggle between StealthChop and SpreadCycle modes
void MotorController::toggleStealthChop() {
    uint32_t currentThreshold = driver.TPWMTHRS();
    if (currentThreshold == 0) {
        // Currently in StealthChop mode, switch to SpreadCycle
        driver.TPWMTHRS(500);  // Switch to SpreadCycle above 500 steps/sec
        Serial.println("Switched to SpreadCycle mode (more power, more noise)");
    } else {
        // Currently in SpreadCycle mode, switch to StealthChop
        driver.TPWMTHRS(0);  // Enable StealthChop mode
        Serial.println("Switched to StealthChop mode (silent operation)");
    }
}

// Performs a basic SPI communication test by sending a test pattern
bool MotorController::testCommunication() {
    uint32_t status = driver.DRV_STATUS();
    if (status != 0xffffffff) {
        Serial.println("Status: 0x");
        Serial.println(status, HEX);
        return true;
    }
    return false;
}

// Activates the SPI device by setting CS pin low
void MotorController::enableSPI() {
    digitalWrite(csPin, LOW);
}

// Deactivates the SPI device by setting CS pin high
void MotorController::disableSPI() {
    digitalWrite(csPin, HIGH);
}

// Performs a single byte SPI transfer
uint8_t MotorController::transfer(uint8_t data) {
    return SPI.transfer(data);
}

void MotorController::enableDriver() {
    digitalWrite(enPin, LOW);
}

void MotorController::disableDriver() {
    digitalWrite(enPin, HIGH);
}

void MotorController::stepHigh() {
    digitalWrite(stepPin, HIGH);
}

void MotorController::stepLow() {
    digitalWrite(stepPin, LOW);
}

void MotorController::dirHigh() {
    digitalWrite(dirPin, HIGH);
}

void MotorController::dirLow() {
    digitalWrite(dirPin, LOW);
}