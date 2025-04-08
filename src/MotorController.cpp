#include "MotorController.h"
// driver.TCOOLTHRS(threshold);

// Constructor initializes motor driver and sets default parameters
MotorController::MotorController(const char* name, uint8_t csPin, uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                                 uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin)
    : driver(csPin, mosiPin, misoPin, sckPin),
      instanceName(name),
      csPin(csPin),
      stepPin(stepPin),
      dirPin(dirPin),
      enPin(enPin),
      mosiPin(mosiPin),
      misoPin(misoPin),
      sckPin(sckPin),
      stepDelay(500),
      lastStepTime(0),
      stepCounter(0),
      runCurrent(200),
      holdCurrent(200),
      speed(200),
      maxSpeed(200),
      acceleration(500),
      maxAcceleration(500),
      maxDeceleration(500),
      lastTempPrintTime(0),
      lastTemperature(0),
      coolStepThreshold(1000),
      stallGuardThreshold(10),
      currentHoldDelay(6),
      rampMode(0),  // Default to positioning mode
      isMoving(false),
      direction(true),
      diagnosticsEnabled(false),
      stallGuardFilter(true),
      spreadCycleEnabled(false),
      microstepInterpolation(true)
{
}

// Initialize motor controller and driver
void MotorController::begin()
{
    setupPins();
    disableSPI();
    resetDriverState();
    configureDriver2();
}

// Start motor movement in forward direction
void MotorController::moveForward()
{
    if (!diagnoseTMC5160())
    {
        Serial.println(F("❌ Driver error detected. Motor will not move."));
        return;
    }
    setMovementDirection(true);
}

// Start motor movement in reverse direction
void MotorController::moveReverse()
{
    if (!diagnoseTMC5160())
    {
        Serial.println(F("❌ Driver error detected. Motor will not move."));
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
            // update diagnostics
            if (diagnosticsEnabled)
            {
                uint32_t load_value = driver.sg_result();
                int      temp       = getTemperature();

                Serial.print(F("Diagnostics - "));
                Serial.print(instanceName);
                Serial.print(F(": Load="));
                Serial.print(load_value);
                Serial.print(F(", Temp="));
                Serial.print(temp);
                Serial.println(F("°C"));

                // check Stalled
                uint32_t status = driver.DRV_STATUS();
                if (status & 0x00000200)
                {
                    Serial.println(F("❌ HARD STALL detected! Stopping motor."));
                    stop();  // Stop motor on stall
                    printStallGuardStatus(status);
                }

                // is Stalled?
                if (driver.sg_result() < stallGuardThreshold)
                {
                    // handle Stall
                    Serial.print(F("⚠️ Pre-stall warning on "));
                    Serial.println(instanceName);

                    // Reduce current temporarily
                    uint16_t originalCurrent = runCurrent;
                    driver.rms_current(runCurrent * 0.7);

                    // Wait for recovery
                    delay(100);

                    // Restore current
                    driver.rms_current(originalCurrent);
                }
            }

            lastDiagnosticTime = millis();
        }

        // Check load and optimize current less frequently
        static unsigned long lastLoadCheckTime = 0;
        if (millis() - lastLoadCheckTime >= 50)
        {
            uint16_t load = driver.sg_result();
            if (load > 1500)
            {
                Serial.print(F("⚠️ WARNING: High load detected on "));
                Serial.print(instanceName);
                Serial.print(F(": "));
                Serial.println(load);
            }
            else if (load > 1000)
            {
                // Increase current by 20% but not above MAX_RUN_CURRENT
                uint16_t newCurrent =
                    static_cast<uint16_t>(std::min(static_cast<double>(runCurrent * 1.2), static_cast<double>(1000)));
                driver.rms_current(newCurrent);
                runCurrent = newCurrent;
            }
            else if (load < 1000 / 2)
            {
                // Decrease current by 20% but not below MIN_CURRENT
                uint16_t newCurrent =
                    static_cast<uint16_t>(std::max(static_cast<double>(runCurrent * 0.8), static_cast<double>(100)));
                driver.rms_current(newCurrent);
                runCurrent = newCurrent;
            }

            lastLoadCheckTime = millis();
        }

        // Monitor temperature less frequently
        static unsigned long lastTempCheckTime = 0;
        if (millis() - lastTempCheckTime >= 200)
        {
            int temp = getTemperature();
            if (temp > 80)
            {
                Serial.print(F("⚠️ WARNING: High temperature detected on "));
                Serial.print(instanceName);
                Serial.print(F(": "));
                Serial.println(temp);
                uint16_t reducedCurrent = runCurrent * 0.8;
                driver.rms_current(reducedCurrent);
            }
            lastTempCheckTime = millis();
        }

        // Print temperature at configured interval
        if (millis() - lastTempPrintTime >= 1000)
        {
            printTemperature();
            lastTempPrintTime = millis();
        }

        // Adjust microstepping based on speed
        static unsigned long lastMicrostepCheckTime = 0;
        if (millis() - lastMicrostepCheckTime >= 100)
        {
            // adjust Microstepping
            if (speed > 5000)
            {
                driver.microsteps(8);  // Reduce microstepping at high speeds
            }
            else
            {
                driver.microsteps(16);
            }

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
    if (runCurrent < 1000)
    {
        runCurrent += 100;
        driver.rms_current(runCurrent);
        Serial.print(F("Run current increased to: "));
        Serial.print(runCurrent);
        Serial.println(F("mA (Max: 1000mA)"));
    }
    else
    {
        Serial.println(F("⚠️ Run current at maximum (1000mA)"));
    }
}

void MotorController::decreaseRunCurrent()
{
    if (runCurrent > 100)
    {
        runCurrent -= 100;
        driver.rms_current(runCurrent);
        Serial.print(F("Run current decreased to: "));
        Serial.print(runCurrent);
        Serial.println(F("mA (Min: 100mA)"));
    }
    else
    {
        Serial.println(F("⚠️ Run current at minimum (100mA)"));
    }
}

void MotorController::increaseHoldCurrent()
{
    if (holdCurrent < 500)
    {
        holdCurrent += 100;
        driver.ihold(holdCurrent);
        Serial.print(F("Hold current increased to: "));
        Serial.print(holdCurrent);
        Serial.println(F("mA (Max: 500mA)"));
    }
    else
    {
        Serial.println(F("Hold current at maximum (500mA)"));
    }
}

void MotorController::decreaseHoldCurrent()
{
    if (holdCurrent > 100)
    {
        holdCurrent -= 100;
        driver.ihold(holdCurrent);
        Serial.print(F("Hold current decreased to: "));
        Serial.print(holdCurrent);
        Serial.println(F("mA (Min: 100mA)"));
    }
    else
    {
        Serial.println(F("Hold current at minimum (100mA)"));
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
    if (speed < 10000)
    {
        speed += 100;
        Serial.print(F("Speed increased to: "));
        Serial.print(speed);
        Serial.println(F(" steps/sec"));
    }
    else
    {
        Serial.println(F("Speed at maximum (10000 steps/sec)"));
    }
}

void MotorController::decreaseSpeed()
{
    if (speed > 100)
    {
        speed -= 100;
        Serial.print(F("Speed decreased to: "));
        Serial.print(speed);
        Serial.println(F(" steps/sec"));
    }
    else
    {
        Serial.println(F("Speed at minimum (100 steps/sec)"));
    }
}

void MotorController::increaseAcceleration()
{
    if (acceleration < 10000)
    {
        acceleration += 100;
        driver.AMAX(acceleration);
        Serial.print(F("Acceleration increased to: "));
        Serial.print(acceleration);
        Serial.println(F(" steps/sec²"));
    }
    else
    {
        Serial.println(F("Acceleration at maximum (10000 steps/sec²)"));
    }
}

void MotorController::decreaseAcceleration()
{
    if (acceleration > 100)
    {
        acceleration -= 100;
        driver.AMAX(acceleration);
        Serial.print(F("Acceleration decreased to: "));
        Serial.print(acceleration);
        Serial.println(F(" steps/sec²"));
    }
    else
    {
        Serial.println(F("Acceleration at minimum (100 steps/sec²)"));
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

void MotorController::printDriverStatus()
{
    uint32_t status = driver.DRV_STATUS();

    Serial.println(F("🧠 DRV_STATUS Report"));
    Serial.println(F("──────────────────────────────────────────────"));

    Serial.println(status & (1 << 31) ? F("✅ Standstill (stst)") : F("🌀 Motor moving"));
    Serial.println(status & (1 << 30) ? F("❌ Open load on Phase B (olb)") : F("✅ Phase B OK"));
    Serial.println(status & (1 << 29) ? F("❌ Open load on Phase A (ola)") : F("✅ Phase A OK"));
    Serial.println(status & (1 << 28) ? F("❌ Short to GND on Phase B (s2gb)") : F("✅ Phase B GND OK"));
    Serial.println(status & (1 << 27) ? F("❌ Short to GND on Phase A (s2ga)") : F("✅ Phase A GND OK"));
    Serial.println(status & (1 << 26) ? F("⚠️  Overtemperature pre-warning (otpw)") : F("✅ Temp OK"));
    Serial.println(status & (1 << 25) ? F("🔥 Overtemperature shutdown (ot)") : F("✅ Not overheated"));
    Serial.println(status & (1 << 24) ? F("⚠️  StallGuard: Stall detected!") : F("✅ No stall"));

    Serial.println(status & (1 << 15) ? F("📦 Fullstep active (fsactive)") : F("⏩ Microstepping active"));
    Serial.println(status & (1 << 14) ? F("🎧 StealthChop active (stealth)") : F("⚡ SpreadCycle active"));

    Serial.println(status & (1 << 13) ? F("❌ Short to V+ on Phase B (s2vbs)") : F("✅ Phase B Supply OK"));
    Serial.println(status & (1 << 12) ? F("❌ Short to V+ on Phase A (s2vsa)") : F("✅ Phase A Supply OK"));

    uint8_t cs_actual = (status >> 17) & 0x0F;
    Serial.print(F("CS_ACTUAL (current scaling): "));
    Serial.println(cs_actual);

    float current_mA = cs_actual / 32.0 * driver.rms_current();
    Serial.print(F("Estimated actual current = "));
    Serial.print(current_mA);
    Serial.println(F(" mA"));

    uint16_t sg_result = status & 0x03FF;
    if (sg_result < 100)
    {
        Serial.println(F("⚠️  Possi ble stall condition!"));
    }
    else if (sg_result < 500)
    {
        Serial.println(F("ℹ️  Moderate load"));
    }
    else
    {
        Serial.println(F("✅ Light load"));
    }

    Serial.println(F("──────────────────────────────────────────────"));
}

void MotorController::printDriverConfig()
{
    Serial.println(F("\nDriver Configuration:"));
    Serial.println(F("-------------------"));
    Serial.print(F("  Run Current: "));
    Serial.print(runCurrent);
    Serial.println(F("mA"));
    Serial.print(F("  Hold Current: "));
    Serial.print(holdCurrent);
    Serial.println(F("mA"));
    Serial.print(F("  Microsteps: "));
    Serial.println(16);
    Serial.print(F("  Speed: "));
    Serial.print(speed);
    Serial.println(F(" steps/sec"));
    Serial.print(F("  Acceleration: "));
    Serial.print(acceleration);
    Serial.println(F(" steps/sec²"));

    Serial.println(F("\nDriver Parameters:"));
    Serial.println(F("------------------"));
    Serial.print(F("  GCONF (Global Config): 0x"));
    Serial.println(driver.GCONF(), HEX);
    Serial.print(F("  TPOWERDOWN (Power Down Time): "));
    Serial.print(driver.TPOWERDOWN());
    Serial.println(F(" tclk"));
    Serial.print(F("  TSTEP (Current Step Timing): "));
    Serial.print(driver.TSTEP());
    Serial.println(F(" tclk"));
    Serial.print(F("  TPWMTHRS (StealthChop Threshold): "));
    Serial.print(driver.TPWMTHRS());
    Serial.println(F(" tclk"));
    Serial.print(F("  THIGH (Step Pulse High Time): "));
    Serial.print(driver.THIGH());
    Serial.println(F(" tclk"));
    Serial.print(F("  XDIRECT (Direct Coil Control): 0x"));
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
        Serial.print(F(": "));
        Serial.print(temp);
        Serial.println(F("°C"));
        lastTemperature = temp;
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
        Serial.println(F("⚡ Switched to SpreadCycle mode (more power, more noise)"));
    }
    else
    {
        // Currently in SpreadCycle mode, switch to StealthChop
        driver.TPWMTHRS(0);  // Enable StealthChop mode
        Serial.println(F("✅ Switched to StealthChop mode (silent operation)"));
    }
}

void MotorController::setStealthChopMode(bool enable)
{
    driver.en_pwm_mode(true);  // Ensure StealthChop is available

    if (enable)
    {
        driver.TPWMTHRS(0);  // StealthChop always
        Serial.println(F("✅ StealthChop mode activated (TPWMTHRS = 0)"));
    }
    else
    {
        driver.TPWMTHRS(300);  // SpreadCycle beyond this speed
        Serial.println(F("⚡ SpreadCycle mode activated (TPWMTHRS = 300)"));
    }
}

bool MotorController::diagnoseTMC5160()
{
    uint32_t status = driver.DRV_STATUS();
    bool     ok     = true;

    if (status & (1 << 27))
    {
        Serial.println(F("❌ ERROR: Short to GND on Phase A (s2ga)"));
        ok = false;
    }

    if (status & (1 << 28))
    {
        Serial.println(F("❌ ERROR: Short to GND on Phase B (s2gb)"));
        ok = false;
    }

    if (status & (1 << 12))
    {
        Serial.println(F("❌ ERROR: Short to supply on Phase A (s2vsa)"));
        ok = false;
    }

    if (status & (1 << 13))
    {
        Serial.println(F("❌ ERROR: Short to supply on Phase B (s2vsb)"));
        ok = false;
    }

    if (status & (1 << 25))
    {
        Serial.println(F("🔥 CRITICAL: Overtemperature shutdown active (ot)"));
        ok = false;
    }

    if (status & (1 << 24))
    {
        Serial.println(F("⚠️  WARNING: Motor stall detected (StallGuard)"));
        ok = false;
    }

    if (ok)
    {
        Serial.println(F("✅ All driver diagnostics OK."));
    }

    return ok;
}

// Performs a basic SPI communication test by sending a test pattern
bool MotorController::testCommunication(bool enableMessage)
{
    if (enableMessage)
    {
        Serial.print(F("Testing SPI communication with TMC5160: "));
    }

    enableSPI();

    uint32_t gconf  = driver.GCONF();
    uint32_t status = driver.DRV_STATUS();

    if (gconf == 0xFFFFFFFF || status == 0xFFFFFFFF)
    {
        if (enableMessage)
        {
            Serial.println(F("❌ Failed\n"));
        }
        return false;
    }

    if (enableMessage)
    {
        Serial.println(F("✅ Ok\n"));
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

// Activates the SPI device by setting CS pin low
void MotorController::enableSPI()
{
    digitalWrite(csPin, LOW);
    delay(5);
}

// Deactivates the SPI device by setting CS pin high
void MotorController::disableSPI()
{
    digitalWrite(csPin, HIGH);
    delay(5);
}

void MotorController::resetDriverState()
{
    disableDriver();
    enableDriver();
}

void MotorController::setSpreadCycle(bool enable)
{
    spreadCycleEnabled = enable;
    driver.en_pwm_mode(!enable);  // 0 for spread cycle, 1 for stealthChop
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

// Configure TMC5160 driver parameters for medical-grade precision
void MotorController::configureDriver2()
{
    driver.begin();
    delay(5);

    // ✅ Set GCONF with bitmask to enable important features
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Enable internal RSense
    gconf |= (1 << 2);  // Enable StealthChop
    gconf |= (1 << 3);  // Enable microstep interpolation
    gconf |= (1 << 4);  // Double edge step
    gconf |= (1 << 6);  // Enable multi-step filtering
    driver.GCONF(gconf);
    delay(5);

    // ✅ Fine-tune currents
    driver.rms_current(runCurrent);       // RMS current while running
    driver.ihold(holdCurrent);            // Holding current
    driver.irun(runCurrent);              // Running current
    driver.iholddelay(currentHoldDelay);  // Delay to transition to holding current
    driver.TPOWERDOWN(10);                // Motor shutdown time

    // ✅ Microstepping setting
    driver.microsteps(16);
    driver.intpol(microstepInterpolation);

    // ✅ CoolStep activation and setting
    driver.TCOOLTHRS(coolStepThreshold);  // CoolStep / StallGuard activation threshold
    driver.semin(5);                      // CoolStep activation (value > 0)
    driver.semax(2);                      // Maximum current increase level
    driver.seup(0b01);                    // Current increase rate
    driver.sedn(0b01);                    // Current decrease rate
    driver.sgt(stallGuardThreshold);      // StallGuard sensitivity
    driver.sfilt(stallGuardFilter);       // Enable pager filter (1 = filter on)

    // ✅ Enable StealthChop with PWM tuning
    driver.TPWMTHRS(0);          // StealthChop always on
    driver.pwm_autoscale(true);  // Enable current auto-tuning
    driver.pwm_autograd(true);   // Enable auto-grading
    driver.pwm_ofs(36);
    driver.pwm_grad(14);
    driver.pwm_freq(1);

    // ✅ Select current control mode (StealthChop vs SpreadCycle)
    driver.en_pwm_mode(!spreadCycleEnabled);  // true = StealthChop, false = SpreadCycle

    // ✅ SpreadCycle settings (if Spread is enabled)
    driver.toff(5);  // Chopper activation
    driver.blank_time(24);
    driver.hysteresis_start(5);
    driver.hysteresis_end(3);

    // ✅ Motion control (acceleration and speed profile)
    driver.RAMPMODE(rampMode);
    driver.VSTART(0);       // Start from zero speed
    driver.VMAX(maxSpeed);  // Maximum speed
    driver.VSTOP(10);       // Soft stop, recommended: 5–10
    driver.AMAX(maxAcceleration);
    driver.DMAX(maxDeceleration);
    driver.a1(maxAcceleration);
    driver.v1(maxSpeed / 2);
    driver.d1(maxDeceleration);

    // ✅ Final driver activation
    enableDriver();
    delay(5);
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
    driver.TPOWERDOWN(10);

    // Configure microstepping
    driver.microsteps(16);
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
    driver.pwm_ofs(36);
    driver.pwm_grad(14);
    driver.pwm_freq(1);

    // Configure spreadCycle
    driver.en_pwm_mode(!spreadCycleEnabled);  // 0 for spread cycle, 1 for stealthChop
    driver.toff(5);
    driver.blank_time(24);
    driver.hysteresis_start(5);
    driver.hysteresis_end(3);

    // Configure motion control
    driver.RAMPMODE(rampMode);
    driver.VMAX(maxSpeed);
    driver.AMAX(maxAcceleration);
    driver.DMAX(maxDeceleration);
    driver.a1(maxAcceleration);
    driver.v1(maxSpeed / 2);
    driver.d1(maxDeceleration);
    driver.VSTART(0);
    driver.VSTOP(5);

    enableDriver();
    delay(5);
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

// Execute a single step
void MotorController::step()
{
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    lastStepTime = micros();
    stepCounter  = (stepCounter + 1) % 1000;
}

// Check driver status and reinitialize if needed
bool MotorController::checkAndReinitializeDriver()
{
    uint32_t status = driver.DRV_STATUS();
    if (status == 0 || status == 0xFFFFFFFF)
    {
        // handle PowerLoss
        disableDriver();
        disableSPI();
        enableSPI();

        // Reconfigure driver
        configureDriver2();
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
    Serial.print(F("Moving "));
    Serial.println(forward ? F("Forward") : F("Reverse"));
}

// Status printing helper methods
void MotorController::printStatusRegister(uint32_t status)
{
    Serial.println(F("\nDriver Status Register:"));
    Serial.print(F("Raw Status: 0x"));
    Serial.println(status, HEX);
    printErrorFlags(status);
    printStallGuardStatus(status);
    printDriverState(status);
}

void MotorController::printErrorFlags(uint32_t status)
{
    Serial.println(F("\nError Flags:"));
    Serial.print(F("  Over Temperature: "));
    Serial.println((status & 0x00000001) ? F("Yes") : F("No"));
    Serial.print(F("  Short to Ground A: "));
    Serial.println((status & 0x00000002) ? F("Yes") : F("No"));
    Serial.print(F("  Short to Ground B: "));
    Serial.println((status & 0x00000004) ? F("Yes") : F("No"));
    Serial.print(F("  Open Load A: "));
    Serial.println((status & 0x00000008) ? F("Yes") : F("No"));
    Serial.print(F("  Open Load B: "));
    Serial.println((status & 0x00000010) ? F("Yes") : F("No"));
}

void MotorController::printStallGuardStatus(uint32_t status)
{
    Serial.println(F("\nStallGuard Status:"));
    Serial.print(F("  StallGuard Value: "));
    Serial.println((status >> 10) & 0x3FF);
    Serial.print(F("  Stall Detected: "));
    Serial.println((status & 0x00000200) ? F("Yes") : F("No"));
}

void MotorController::printDriverState(uint32_t status)
{
    Serial.println(F("\nDriver State:"));
    Serial.print(F("  Standstill: "));
    Serial.println((status & 0x00000400) ? F("Yes") : F("No"));
    Serial.print(F("  Velocity Reached: "));
    Serial.println((status & 0x00000800) ? F("Yes") : F("No"));
    Serial.print(F("  Position Reached: "));
    Serial.println((status & 0x00001000) ? F("Yes") : F("No"));
}
