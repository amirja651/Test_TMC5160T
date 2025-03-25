#include "MotorController.h"
#include "motor_specs.h"
MotorController& MotorController::getInstance() {
    static MotorController instance;
    return instance;
}

MotorController::MotorController()
    : driver(Config::SPI::CS, Config::SPI::MOSI, Config::SPI::MISO, Config::SPI::SCK),
      isMoving(false),
      direction(true),
      stepDelay(500),
      lastStepTime(0),
      stepCounter(0),
      runCurrent(MOTOR_P28SHD4611_12SK.operation.runCurrent),      // Default 1000mA
      holdCurrent(MOTOR_P28SHD4611_12SK.operation.holdCurrent),    // Default 500mA
      speed(MOTOR_P28SHD4611_12SK.operation.speed),                // Default 1000 steps/sec
      acceleration(MOTOR_P28SHD4611_12SK.operation.acceleration),  // Default 1000 steps/sec²
      lastTempPrintTime(0),
      lastTemperature(0) {}

void MotorController::begin() {
    setupPins();
    delay(100);  // Wait for power to stabilize

    // Reset driver state
    digitalWrite(Config::TMC5160T_Driver::EN_PIN, HIGH);
    delay(100);
    digitalWrite(Config::TMC5160T_Driver::EN_PIN, LOW);
    delay(100);

    configureDriver();
}

void MotorController::setupPins() {
    pinMode(Config::TMC5160T_Driver::STEP_PIN, OUTPUT);
    pinMode(Config::TMC5160T_Driver::DIR_PIN, OUTPUT);
    pinMode(Config::TMC5160T_Driver::EN_PIN, OUTPUT);

    digitalWrite(Config::TMC5160T_Driver::EN_PIN, HIGH);
    delay(200);
}

void MotorController::configureDriver() {
    driver.begin();
    delay(200);

    driver.GCONF(0x0);
    delay(100);

    // Set initial currents
    driver.rms_current(runCurrent);
    driver.ihold(holdCurrent);
    delay(100);

    driver.microsteps(16);
    delay(100);

    // Configure CoolStep
    driver.TPWMTHRS(0);      // Enable StealthChop mode by default
    driver.TCOOLTHRS(1000);  // CoolStep threshold
    driver.TPOWERDOWN(10);   // Power down time after standstill

    // Basic driver settings
    driver.toff(5);
    driver.blank_time(24);
    driver.iholddelay(6);
    delay(100);

    digitalWrite(Config::TMC5160T_Driver::EN_PIN, LOW);
    delay(200);
}

void MotorController::handlePowerLoss() {
    // Disable motor
    digitalWrite(Config::TMC5160T_Driver::EN_PIN, HIGH);
    delay(100);

    // Reset SPI communication
    SPIManager::getInstance().deselectDevice();
    delay(100);
    SPIManager::getInstance().selectDevice();
    delay(100);

    // Reconfigure driver
    configureDriver();
}

bool MotorController::checkAndReinitializeDriver() {
    uint32_t status = driver.DRV_STATUS();
    if (status == 0 || status == INVALID_STATUS) {
        handlePowerLoss();
        return true;
    }
    return false;
}

void MotorController::moveForward() {
    uint32_t status = driver.DRV_STATUS();
    if (status & 0x00000001) {  // Check for drive voltage error (bit 0)
        Serial.println("ERROR: Drive voltage error detected. Movement command ignored.");
        Serial.println("Please check power supply and connections.");
        return;
    }

    isMoving  = true;
    direction = true;
    digitalWrite(Config::TMC5160T_Driver::DIR_PIN, direction ? HIGH : LOW);
    Serial.println("Moving Forward");
}

void MotorController::moveReverse() {
    uint32_t status = driver.DRV_STATUS();
    if (status & 0x00000001) {  // Check for drive voltage error (bit 0)
        Serial.println("ERROR: Drive voltage error detected. Movement command ignored.");
        Serial.println("Please check power supply and connections.");
        return;
    }

    isMoving  = true;
    direction = false;
    digitalWrite(Config::TMC5160T_Driver::DIR_PIN, direction ? HIGH : LOW);
    Serial.println("Moving Reverse");
}

void MotorController::stop() {
    isMoving = false;
}

void MotorController::step() {
    digitalWrite(Config::TMC5160T_Driver::STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(Config::TMC5160T_Driver::STEP_PIN, LOW);
    lastStepTime = micros();

    if (++stepCounter >= STATUS_PRINT_INTERVAL) {
        stepCounter = 0;
    }
}

void MotorController::update() {
    if (isMoving) {
        if (micros() - lastStepTime >= (1000000 / speed)) {
            step();
        }

        // Check for stall condition
        checkStall();

        // Monitor temperature and adjust current if needed
        int temp = getTemperature();
        if (temp > 60) {  // Temperature warning threshold
            Serial.print("WARNING: High temperature detected: ");
            Serial.println(temp);
            // Reduce current by 20% if temperature is high
            uint16_t reducedCurrent = runCurrent * 0.8;
            driver.rms_current(reducedCurrent);
            Serial.print("Reduced current to: ");
            Serial.println(reducedCurrent);
        }

        // Print temperature every second
        if (millis() - lastTempPrintTime >= TEMP_PRINT_INTERVAL) {
            printTemperature();
            lastTempPrintTime = millis();
        }
    }
}

uint32_t MotorController::getDriverStatus() {
    return driver.DRV_STATUS();
}

void MotorController::increaseRunCurrent() {
    if (runCurrent < MAX_RUN_CURRENT) {
        runCurrent += CURRENT_STEP;
        driver.rms_current(runCurrent);
        Serial.print("Run current increased to: ");
        Serial.print(runCurrent);
        Serial.println("mA (Max: 1000mA)");
    } else {
        Serial.println("Run current at maximum (1000mA)");
    }
}

void MotorController::decreaseRunCurrent() {
    if (runCurrent > MIN_CURRENT) {
        runCurrent -= CURRENT_STEP;
        driver.rms_current(runCurrent);
        Serial.print("Run current decreased to: ");
        Serial.print(runCurrent);
        Serial.println("mA (Min: 100mA)");
    } else {
        Serial.println("Run current at minimum (100mA)");
    }
}

void MotorController::increaseHoldCurrent() {
    if (holdCurrent < MAX_HOLD_CURRENT) {
        holdCurrent += CURRENT_STEP;
        driver.ihold(holdCurrent);
        Serial.print("Hold current increased to: ");
        Serial.print(holdCurrent);
        Serial.println("mA (Max: 500mA)");
    } else {
        Serial.println("Hold current at maximum (500mA)");
    }
}

void MotorController::decreaseHoldCurrent() {
    if (holdCurrent > MIN_CURRENT) {
        holdCurrent -= CURRENT_STEP;
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
    if (speed < MAX_SPEED) {
        speed += SPEED_STEP;
        Serial.print("Speed increased to: ");
        Serial.print(speed);
        Serial.println(" steps/sec");
    } else {
        Serial.println("Speed at maximum (10000 steps/sec)");
    }
}

void MotorController::decreaseSpeed() {
    if (speed > MIN_SPEED) {
        speed -= SPEED_STEP;
        Serial.print("Speed decreased to: ");
        Serial.print(speed);
        Serial.println(" steps/sec");
    } else {
        Serial.println("Speed at minimum (100 steps/sec)");
    }
}

void MotorController::increaseAcceleration() {
    if (acceleration < MAX_ACCEL) {
        acceleration += ACCEL_STEP;
        driver.AMAX(acceleration);
        Serial.print("Acceleration increased to: ");
        Serial.print(acceleration);
        Serial.println(" steps/sec²");
    } else {
        Serial.println("Acceleration at maximum (10000 steps/sec²)");
    }
}

void MotorController::decreaseAcceleration() {
    if (acceleration > MIN_ACCEL) {
        acceleration -= ACCEL_STEP;
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

void MotorController::checkStall() {
    uint32_t status = driver.DRV_STATUS();
    if (status & 0x00000200) {  // Check stall bit
        Serial.println("WARNING: Stall detected!");
        stop();  // Stop motor on stall
        printStallGuardStatus(status);
    }
}

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