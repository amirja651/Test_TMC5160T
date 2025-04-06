#include "MotorController.h"

// Constructor initializes motor driver and sets default parameters
MotorController::MotorController(const char* name, uint8_t csPin, uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
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
      lastTemperature(0),
      instanceName(name),
      diagnosticsEnabled(false),
      coolStepThreshold(Config::TMC5160T_Driver::TCOOLTHRS),
      stallGuardThreshold(Config::TMC5160T_Driver::SGTHRS),
      stallGuardFilter(true),
      spreadCycleEnabled(false),
      microstepInterpolation(true),
      currentScaling(Config::TMC5160T_Driver::CURRENT_SCALING),
      currentHoldDelay(Config::TMC5160T_Driver::IHOLDDELAY),
      currentRunDelay(Config::TMC5160T_Driver::IRUNDELAY),
      rampMode(0),  // Default to positioning mode
      maxSpeed(Config::MotorSpecs::Operation::SPEED),
      maxAcceleration(Config::MotorSpecs::Operation::ACCELERATION),
      maxDeceleration(Config::MotorSpecs::Operation::ACCELERATION)
{
}

// Initialize motor controller and driver
void MotorController::begin()
{
    setupPins();
    disableSPI();
    resetDriverState();
    configureDriver();
    optimizeForPancake();
}

// Configure GPIO pins for motor control
void MotorController::setupPins()
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    pinMode(csPin, OUTPUT);
    delay(5);
}

// Deactivates the SPI device by setting CS pin high
void MotorController::disableSPI()
{
    digitalWrite(csPin, HIGH);
    delay(5);
}

// Activates the SPI device by setting CS pin low
void MotorController::enableSPI()
{
    digitalWrite(csPin, LOW);
    delay(5);
}

void MotorController::resetDriverState()
{
    disableDriver();
    enableDriver();
}

// Configure TMC5160 driver parameters for medical-grade precision
void MotorController::configureDriver()
{
    driver.begin();
    delay(5);

    // Configure GCONF register for optimal performance
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Enable internal RSense
    gconf |= (1 << 2);  // Enable stealthChop
    gconf |= (1 << 3);  // Enable microstep interpolation
    gconf |= (1 << 4);  // Enable double edge step
    gconf |= (1 << 6);  // Enable multistep filtering
    driver.GCONF(gconf);
    delay(5);

    // Set current control parameters
    driver.rms_current(runCurrent);
    driver.ihold(holdCurrent);
    driver.irun(runCurrent);
    driver.iholddelay(currentHoldDelay);
    driver.TPOWERDOWN(Config::TMC5160T_Driver::TPOWERDOWN);

    // Configure microstepping
    driver.microsteps(Config::TMC5160T_Driver::MICROSTEPS);
    driver.intpol(microstepInterpolation);

    // Configure CoolStep
    driver.TCOOLTHRS(coolStepThreshold);
    driver.sgt(stallGuardThreshold);
    driver.sfilt(stallGuardFilter);
    driver.sgt(stallGuardThreshold);

    // Configure stealthChop
    driver.TPWMTHRS(0);  // Enable stealthChop by default
    driver.pwm_autoscale(true);
    driver.pwm_autograd(true);
    driver.pwm_ofs(Config::TMC5160T_Driver::PWM_OFS);
    driver.pwm_grad(Config::TMC5160T_Driver::PWM_GRAD);
    driver.pwm_freq(Config::TMC5160T_Driver::PWM_FREQ);

    // Configure spreadCycle
    driver.en_pwm_mode(!spreadCycleEnabled);  // 0 for spread cycle, 1 for stealthChop
    driver.toff(Config::TMC5160T_Driver::TOFF);
    driver.blank_time(Config::TMC5160T_Driver::BLANK_TIME);
    driver.hysteresis_start(Config::TMC5160T_Driver::HSTRT);
    driver.hysteresis_end(Config::TMC5160T_Driver::HEND);

    // Configure motion control
    driver.RAMPMODE(rampMode);
    driver.VMAX(maxSpeed);
    driver.AMAX(maxAcceleration);
    driver.DMAX(maxDeceleration);
    driver.a1(maxAcceleration);
    driver.v1(maxSpeed / 2);
    driver.d1(maxDeceleration);
    driver.VSTART(0);
    driver.VSTOP(10);

    enableDriver();
    delay(5);
}

// Handle power loss by reinitializing driver
void MotorController::handlePowerLoss()
{
    disableDriver();
    disableSPI();
    enableSPI();

    // Reconfigure driver
    configureDriver();
}

// Check driver status and reinitialize if needed
bool MotorController::checkAndReinitializeDriver()
{
    uint32_t status = driver.DRV_STATUS();
    if (status == 0 || status == Config::MotorController::INVALID_STATUS)
    {
        handlePowerLoss();
        return true;
    }
    return false;
}

bool MotorController::checkDriveVoltageError(bool enableMessage)
{
    uint32_t status = driver.DRV_STATUS();
    if (status & Config::TMC5160T_Driver::VOLTAGE_BIT_MASK)
    {  // Check for drive voltage error (bit 0)
        if (enableMessage)
        {
            Serial.println("ERROR: Drive voltage error detected ⚠️");
        }
        return true;
    }
    return false;
}

void MotorController::setMovementDirection(bool forward)
{
    isMoving  = true;
    direction = forward;
    digitalWrite(dirPin, direction ? HIGH : LOW);
    delay(5);
    Serial.print("Moving ");
    Serial.println(forward ? "Forward" : "Reverse");
}

// Start motor movement in forward direction
void MotorController::moveForward()
{
    if (checkDriveVoltageError())
    {
        return;
    }
    setMovementDirection(true);
}

// Start motor movement in reverse direction
void MotorController::moveReverse()
{
    if (checkDriveVoltageError())
    {
        return;
    }
    setMovementDirection(false);
}

// Stop motor movement
void MotorController::stop()
{
    isMoving = false;
    driver.ihold(100);  // Reduce to ultra low hold current
}

// Execute a single step
void MotorController::step()
{
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    lastStepTime = micros();

    if (++stepCounter >= Config::MotorController::STATUS_PRINT_INTERVAL)
    {
        stepCounter = 0;
    }
}

// Main update loop for motor control
void MotorController::update()
{
    if (isMoving)
    {
        // Handle step timing
        if (micros() - lastStepTime >= (1000000 / speed))
        {
            step();
        }

        // Update diagnostics less frequently
        static unsigned long lastDiagnosticTime = 0;
        if (diagnosticsEnabled && millis() - lastDiagnosticTime >= 100)
        {
            updateDiagnostics();
            lastDiagnosticTime = millis();
        }

        // Check load and optimize current less frequently
        static unsigned long lastLoadCheckTime = 0;
        if (millis() - lastLoadCheckTime >= 50)
        {
            checkLoad();
            optimizeCurrent();
            lastLoadCheckTime = millis();
        }

        // Monitor temperature less frequently
        static unsigned long lastTempCheckTime = 0;
        if (millis() - lastTempCheckTime >= 200)
        {
            int temp = getTemperature();
            if (temp > Config::TMC5160T_Driver::TEMP_WARNING_THRESHOLD)
            {
                Serial.print("⚠️ WARNING: High temperature detected on ");
                Serial.print(instanceName);
                Serial.print(": ");
                Serial.println(temp);
                uint16_t reducedCurrent = runCurrent * 0.8;
                driver.rms_current(reducedCurrent);
            }
            lastTempCheckTime = millis();
        }

        // Print temperature at configured interval
        if (millis() - lastTempPrintTime >= Config::TMC5160T_Driver::TEMP_PRINT_INTERVAL)
        {
            printTemperature();
            lastTempPrintTime = millis();
        }

        // Adjust microstepping based on speed
        static unsigned long lastMicrostepCheckTime = 0;
        if (millis() - lastMicrostepCheckTime >= 100)
        {
            adjustMicrostepping();
            lastMicrostepCheckTime = millis();
        }
    }
}

uint32_t MotorController::getDriverStatus()
{
    return driver.DRV_STATUS();
}

void MotorController::increaseRunCurrent()
{
    if (runCurrent < Config::MotorController::MAX_RUN_CURRENT)
    {
        runCurrent += Config::MotorController::CURRENT_STEP;
        driver.rms_current(runCurrent);
        Serial.print("Run current increased to: ");
        Serial.print(runCurrent);
        Serial.println("mA (Max: 1000mA)");
    }
    else
    {
        Serial.println("⚠️ Run current at maximum (1000mA)");
    }
}

void MotorController::decreaseRunCurrent()
{
    if (runCurrent > Config::MotorController::MIN_CURRENT)
    {
        runCurrent -= Config::MotorController::CURRENT_STEP;
        driver.rms_current(runCurrent);
        Serial.print("Run current decreased to: ");
        Serial.print(runCurrent);
        Serial.println("mA (Min: 100mA)");
    }
    else
    {
        Serial.println("⚠️ Run current at minimum (100mA)");
    }
}

void MotorController::increaseHoldCurrent()
{
    if (holdCurrent < Config::MotorController::MAX_HOLD_CURRENT)
    {
        holdCurrent += Config::MotorController::CURRENT_STEP;
        driver.ihold(holdCurrent);
        Serial.print("Hold current increased to: ");
        Serial.print(holdCurrent);
        Serial.println("mA (Max: 500mA)");
    }
    else
    {
        Serial.println("Hold current at maximum (500mA)");
    }
}

void MotorController::decreaseHoldCurrent()
{
    if (holdCurrent > Config::MotorController::MIN_CURRENT)
    {
        holdCurrent -= Config::MotorController::CURRENT_STEP;
        driver.ihold(holdCurrent);
        Serial.print("Hold current decreased to: ");
        Serial.print(holdCurrent);
        Serial.println("mA (Min: 100mA)");
    }
    else
    {
        Serial.println("Hold current at minimum (100mA)");
    }
}

uint16_t MotorController::getRunCurrent() const
{
    return runCurrent;
}

uint16_t MotorController::getHoldCurrent() const
{
    return holdCurrent;
}

void MotorController::increaseSpeed()
{
    if (speed < Config::MotorController::MAX_SPEED)
    {
        speed += Config::MotorController::SPEED_STEP;
        Serial.print("Speed increased to: ");
        Serial.print(speed);
        Serial.println(" steps/sec");
    }
    else
    {
        Serial.println("Speed at maximum (10000 steps/sec)");
    }
}

void MotorController::decreaseSpeed()
{
    if (speed > Config::MotorController::MIN_SPEED)
    {
        speed -= Config::MotorController::SPEED_STEP;
        Serial.print("Speed decreased to: ");
        Serial.print(speed);
        Serial.println(" steps/sec");
    }
    else
    {
        Serial.println("Speed at minimum (100 steps/sec)");
    }
}

void MotorController::increaseAcceleration()
{
    if (acceleration < Config::MotorController::MAX_ACCEL)
    {
        acceleration += Config::MotorController::ACCEL_STEP;
        driver.AMAX(acceleration);
        Serial.print("Acceleration increased to: ");
        Serial.print(acceleration);
        Serial.println(" steps/sec²");
    }
    else
    {
        Serial.println("Acceleration at maximum (10000 steps/sec²)");
    }
}

void MotorController::decreaseAcceleration()
{
    if (acceleration > Config::MotorController::MIN_ACCEL)
    {
        acceleration -= Config::MotorController::ACCEL_STEP;
        driver.AMAX(acceleration);
        Serial.print("Acceleration decreased to: ");
        Serial.print(acceleration);
        Serial.println(" steps/sec²");
    }
    else
    {
        Serial.println("Acceleration at minimum (100 steps/sec²)");
    }
}

uint16_t MotorController::getSpeed() const
{
    return speed;
}

uint16_t MotorController::getAcceleration() const
{
    return acceleration;
}

void MotorController::printStatusRegister(uint32_t status)
{
    Serial.println("\nDriver Status Register:");
    Serial.print("Raw Status: 0x");
    Serial.println(status, HEX);
    printErrorFlags(status);
    printStallGuardStatus(status);
    printDriverState(status);
}

void MotorController::printErrorFlags(uint32_t status)
{
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

void MotorController::printStallGuardStatus(uint32_t status)
{
    Serial.println("\nStallGuard Status:");
    Serial.print("  StallGuard Value: ");
    Serial.println((status >> 10) & 0x3FF);
    Serial.print("  Stall Detected: ");
    Serial.println((status & 0x00000200) ? "Yes" : "No");
}

void MotorController::printDriverState(uint32_t status)
{
    Serial.println("\nDriver State:");
    Serial.print("  Standstill: ");
    Serial.println((status & 0x00000400) ? "Yes" : "No");
    Serial.print("  Velocity Reached: ");
    Serial.println((status & 0x00000800) ? "Yes" : "No");
    Serial.print("  Position Reached: ");
    Serial.println((status & 0x00001000) ? "Yes" : "No");
}

void MotorController::printDriverStatus()
{
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
    Serial.println((status & 0x00000001) ? "WARNING - Temperature too high!" : "✅ OK");
    Serial.print("  Short to Ground A: ");
    Serial.println((status & 0x00000002) ? "❌ ERROR - Phase A shorted!" : "✅ OK");
    Serial.print("  Short to Ground B: ");
    Serial.println((status & 0x00000004) ? "❌ ERROR - Phase B shorted!" : "✅ OK");
    Serial.print("  Open Load A: ");
    Serial.println((status & 0x00000008) ? "❌ ERROR - Phase A open!" : "✅ OK");
    Serial.print("  Open Load B: ");
    Serial.println((status & 0x00000010) ? "❌ ERROR - Phase B open!" : "✅ OK");

    // StallGuard Status
    Serial.println("\n3. StallGuard Status:");
    Serial.println("---------------------");
    Serial.print("  StallGuard Value: ");
    Serial.println((status >> 10) & 0x3FF);
    Serial.print("  Stall Detected: ");
    Serial.println((status & 0x00000200) ? "⚠️ WARNING - Motor stalled!" : "✅ OK");

    // Driver State
    Serial.println("\n4. Driver State:");
    Serial.println("----------------");
    Serial.print("  Standstill: ");
    Serial.println((status & 0x00000400) ? "✅ Yes" : "❌ No");
    Serial.print("  Velocity Reached: ");
    Serial.println((status & 0x00000800) ? "✅ Yes" : "❌ No");
    Serial.print("  Position Reached: ");
    Serial.println((status & 0x00001000) ? "✅ Yes" : "❌ No");

    // Temperature
    Serial.println("\n5. Temperature Status:");
    Serial.println("----------------------");
    int temp = getTemperature();
    Serial.print("  Current Temperature: ");
    Serial.print(temp);
    Serial.println("°C");
    if (temp > 60)
    {
        Serial.println("⚠️ WARNING: Temperature above 60°C!");
    }
    else if (temp > 45)
    {
        Serial.println("  NOTE: Temperature is getting high");
    }

    // Operating Mode
    Serial.println("\n6. Operating Mode:");
    Serial.println("------------------");
    Serial.print("  StealthChop Mode: ");
    Serial.println((driver.TPWMTHRS() == 0) ? "✅ Enabled" : "❌ Disabled");
    Serial.print("  SpreadCycle Mode: ");
    Serial.println((driver.TPWMTHRS() > 0) ? "✅ Enabled" : "❌ Disabled");

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

void MotorController::printDriverConfig()
{
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

int MotorController::getTemperature()
{
    uint32_t status  = driver.DRV_STATUS();
    int      rawTemp = (status >> 16) & 0xFF;  // Temperature is in bits 16-23
    return rawTemp;                            // Direct temperature reading in °C (1°C steps)
}

void MotorController::printTemperature()
{
    int temp = getTemperature();
    if (temp != lastTemperature)
    {
        Serial.print(instanceName);
        Serial.print(": ");
        Serial.print(temp);
        Serial.println("°C");
        lastTemperature = temp;
    }
}

// Check for motor stall condition
void MotorController::checkStall()
{
    uint32_t status = driver.DRV_STATUS();
    if (status & Config::TMC5160T_Driver::STALL_BIT_MASK)
    {
        Serial.println("WARNING: Stall detected!");
        stop();  // Stop motor on stall
        printStallGuardStatus(status);
    }
}

// Toggle between StealthChop and SpreadCycle modes
void MotorController::toggleStealthChop()
{
    uint32_t currentThreshold = driver.TPWMTHRS();
    if (currentThreshold == 0)
    {
        // Currently in StealthChop mode, switch to SpreadCycle
        driver.TPWMTHRS(500);  // Switch to SpreadCycle above 500 steps/sec
        Serial.println("Switched to SpreadCycle mode (more power, more noise)");
    }
    else
    {
        // Currently in SpreadCycle mode, switch to StealthChop
        driver.TPWMTHRS(0);  // Enable StealthChop mode
        Serial.println("Switched to StealthChop mode (silent operation)");
    }
}

void MotorController::setStealthChopMode(bool enable)
{
    if (enable)
    {
        driver.TPWMTHRS(0);  // Enable StealthChop full-time
        Serial.println("StealthChop enabled");
    }
    else
    {
        driver.TPWMTHRS(300);  // Enable SpreadCycle above threshold
        Serial.println("SpreadCycle enabled (TPWMTHRS = 300)");
    }
}

// Performs a basic SPI communication test by sending a test pattern
bool MotorController::testCommunication(bool enableMessage)
{
    if (enableMessage)
    {
        Serial.print("Testing SPI communication with TMC5160: ");
    }

    enableSPI();

    uint32_t gconf  = driver.GCONF();
    uint32_t status = driver.DRV_STATUS();

    if (gconf == 0xFFFFFFFF || status == 0xFFFFFFFF)
    {
        if (enableMessage)
        {
            Serial.println("❌ Failed\n");
        }
        return false;
    }

    if (enableMessage)
    {
        Serial.print("✅ Ok\n");
        // Serial.print("  ✅ GCONF: 0x");
        // Serial.println(gconf, HEX);
        // Serial.print("  ✅ DRV_STATUS: 0x");
        // Serial.println(status, HEX);
    }

    return true;
}

// Performs a single byte SPI transfer
uint8_t MotorController::transfer(uint8_t data)
{
    return SPI.transfer(data);
}

void MotorController::enableDriver()
{
    digitalWrite(enPin, LOW);
}

void MotorController::disableDriver()
{
    digitalWrite(enPin, HIGH);
}

void MotorController::stepHigh()
{
    digitalWrite(stepPin, HIGH);
}

void MotorController::stepLow()
{
    digitalWrite(stepPin, LOW);
}

void MotorController::dirHigh()
{
    digitalWrite(dirPin, HIGH);
}

void MotorController::dirLow()
{
    digitalWrite(dirPin, LOW);
}

void MotorController::printInstanceName() const
{
    Serial.print("Motor Controller Instance: ");
    Serial.println(instanceName);
}

// Optimize for pancake motor
void MotorController::optimizeForPancake()
{
    // Set optimal parameters for pancake motor
    setMicrostepInterpolation(true);
    setStallGuardFilter(true);
    setStallGuardThreshold(Config::TMC5160T_Driver::SGTHRS);
    setCoolStepThreshold(Config::TMC5160T_Driver::TCOOLTHRS);

    // Configure for high precision
    driver.microsteps(256);  // Maximum microstepping for smooth motion
    driver.intpol(true);     // Enable microstep interpolation

    // Optimize current control
    setCurrentScaling(Config::TMC5160T_Driver::CURRENT_SCALING);
    setCurrentHoldDelay(Config::TMC5160T_Driver::IHOLDDELAY);
    setCurrentRunDelay(Config::TMC5160T_Driver::IRUNDELAY);

    // Configure motion control
    setRampMode(0);  // Positioning mode for precise control
    setMaxSpeed(Config::MotorSpecs::Operation::MAX_SPEED);
    setMaxAcceleration(Config::MotorSpecs::Operation::MAX_ACCELERATION);
    setMaxDeceleration(Config::MotorSpecs::Operation::MAX_DECELERATION);
}

// Advanced motor control methods
void MotorController::setCoolStepThreshold(uint32_t threshold)
{
    coolStepThreshold = threshold;
    driver.TCOOLTHRS(threshold);
}

void MotorController::setStallGuardThreshold(int8_t threshold)
{
    stallGuardThreshold = threshold;
    driver.sgt(threshold);
}

void MotorController::setStallGuardFilter(bool enable)
{
    stallGuardFilter = enable;
    driver.sfilt(enable);
}

void MotorController::setSpreadCycle(bool enable)
{
    spreadCycleEnabled = enable;
    driver.en_pwm_mode(!enable);  // 0 for spread cycle, 1 for stealthChop
}

void MotorController::setMicrostepInterpolation(bool enable)
{
    microstepInterpolation = enable;
    driver.intpol(enable);
}

// Advanced current control
void MotorController::setCurrentScaling(uint8_t scaling)
{
    currentScaling = scaling;
    driver.ihold(holdCurrent * scaling / 32);
    driver.irun(runCurrent * scaling / 32);
}

void MotorController::setCurrentHoldDelay(uint8_t delay)
{
    currentHoldDelay = delay;
    driver.iholddelay(delay);
}

void MotorController::setCurrentRunDelay(uint8_t delay)
{
    currentRunDelay = delay;
    // Run current delay is handled by irun(runCurrent)
}

// Motion control
void MotorController::setRampMode(uint8_t mode)
{
    rampMode = mode;
    driver.RAMPMODE(mode);
}

void MotorController::setMaxSpeed(uint32_t speed)
{
    maxSpeed = speed;
    driver.VMAX(speed);
}

void MotorController::setMaxAcceleration(uint32_t accel)
{
    maxAcceleration = accel;
    driver.a1(accel);
}

void MotorController::setMaxDeceleration(uint32_t decel)
{
    maxDeceleration = decel;
    driver.d1(decel);
}

// Advanced diagnostics
void MotorController::enableDiagnostics()
{
    diagnosticsEnabled = true;
}

void MotorController::disableDiagnostics()
{
    diagnosticsEnabled = false;
}

uint32_t MotorController::getLoadValue()
{
    return driver.sg_result();
}

bool MotorController::isStalled()
{
    return (driver.sg_result() < stallGuardThreshold);
}

// Update diagnostic information
void MotorController::updateDiagnostics()
{
    if (!diagnosticsEnabled)
        return;

    uint32_t load_value = getLoadValue();
    int      temp       = getTemperature();

    Serial.print("Diagnostics - ");
    Serial.print(instanceName);
    Serial.print(": Load=");
    Serial.print(load_value);
    Serial.print(", Temp=");
    Serial.print(temp);
    Serial.println("°C");

    if (isStalled())
    {
        handleStall();
    }
}

// Handle stall condition
void MotorController::handleStall()
{
    Serial.print("⚠️ WARNING: Stall detected on ");
    Serial.println(instanceName);

    // Reduce current temporarily
    uint16_t originalCurrent = runCurrent;
    driver.rms_current(runCurrent * 0.7);

    // Wait for recovery
    delay(100);

    // Restore current
    driver.rms_current(originalCurrent);
}

// Optimize current based on load
void MotorController::optimizeCurrent()
{
    uint32_t load = getLoadValue();
    if (load > Config::TMC5160T_Driver::LOAD_THRESHOLD)
    {
        // Increase current by 20% but not above MAX_RUN_CURRENT
        uint16_t newCurrent = static_cast<uint16_t>(std::min(
            static_cast<double>(runCurrent * 1.2), static_cast<double>(Config::MotorController::MAX_RUN_CURRENT)));
        driver.rms_current(newCurrent);
        runCurrent = newCurrent;
    }
    else if (load < Config::TMC5160T_Driver::LOAD_THRESHOLD / 2)
    {
        // Decrease current by 20% but not below MIN_CURRENT
        uint16_t newCurrent = static_cast<uint16_t>(
            std::max(static_cast<double>(runCurrent * 0.8), static_cast<double>(Config::MotorController::MIN_CURRENT)));
        driver.rms_current(newCurrent);
        runCurrent = newCurrent;
    }
}

// Check motor load
void MotorController::checkLoad()
{
    uint32_t load = getLoadValue();
    if (load > Config::TMC5160T_Driver::LOAD_WARNING_THRESHOLD)
    {
        Serial.print("⚠️ WARNING: High load detected on ");
        Serial.print(instanceName);
        Serial.print(": ");
        Serial.println(load);
        optimizeCurrent();
    }
}

// Adjust microstepping based on speed
void MotorController::adjustMicrostepping()
{
    if (speed > Config::MotorSpecs::Operation::HIGH_SPEED_THRESHOLD)
    {
        driver.microsteps(8);  // Reduce microstepping at high speeds
    }
    else
    {
        driver.microsteps(Config::TMC5160T_Driver::MICROSTEPS);
    }
}