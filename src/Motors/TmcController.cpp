#include "Motors/TmcController.h"

MotionSystem::TmcController::TmcController(const char* name, uint8_t csPin, uint8_t stepPin, uint8_t dirPin,
                                           uint8_t enPin, uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin)
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

void MotionSystem::TmcController::begin()
{
    setupPins();
    disableSPI();
    resetDriverState();
    configureDriver2();
}

void MotionSystem::TmcController::moveForward()
{
    if (!diagnoseTMC5160())
    {
        Logger::getInstance().logln(F("❌ Driver error detected. Motor will not move."));
        return;
    }

    setDirection(true);
}

void MotionSystem::TmcController::moveReverse()
{
    if (!diagnoseTMC5160())
    {
        Logger::getInstance().logln(F("❌ Driver error detected. Motor will not move."));
        return;
    }

    setDirection(false);
}

void MotionSystem::TmcController::stop()
{
    isMoving = false;
    driver.ihold(100);  // Reduce to ultra low hold current
}

void MotionSystem::TmcController::update()
{
    if (isMoving)
    {
        if (micros() - lastStepTime >= (1000000 / speed))
        {
            step();
        }

        static unsigned long lastDiagnosticTime = 0;
        if (diagnosticsEnabled && millis() - lastDiagnosticTime >= 100)
        {
            if (diagnosticsEnabled)
            {
                uint32_t load_value = driver.sg_result();
                int      temp       = getTemperature();
                Logger::getInstance().log(F("Diagnostics - "));
                Logger::getInstance().log(instanceName);
                Logger::getInstance().log(F(": Load="));
                Logger::getInstance().log(String(load_value));
                Logger::getInstance().log(F(", Temp="));
                Logger::getInstance().log(String(temp));
                Logger::getInstance().logln(F("°C"));
                uint32_t status = driver.DRV_STATUS();
                if (status & 0x00000200)
                {
                    Logger::getInstance().logln(F("❌ HARD STALL detected! Stopping motor."));
                    stop();  // Stop motor on stall
                    printStallGuardStatus(status);
                }

                if (driver.sg_result() < stallGuardThreshold)
                {
                    Logger::getInstance().log(F("⚠️ Pre-stall warning on "));
                    Logger::getInstance().logln(instanceName);
                    uint16_t originalCurrent = runCurrent;
                    driver.rms_current(runCurrent * 0.7);
                    delay(100);
                    driver.rms_current(originalCurrent);
                }
            }

            lastDiagnosticTime = millis();
        }

        static unsigned long lastLoadCheckTime = 0;
        if (millis() - lastLoadCheckTime >= 50)
        {
            uint16_t load = driver.sg_result();
            if (load > 1500)
            {
                Logger::getInstance().log(F("⚠️ WARNING: High load detected on "));
                Logger::getInstance().log(instanceName);
                Logger::getInstance().log(F(": "));
                Logger::getInstance().logln(String(load));
            }

            else if (load > 1000)
            {
                uint16_t newCurrent =
                    static_cast<uint16_t>(std::min(static_cast<double>(runCurrent * 1.2), static_cast<double>(1000)));
                driver.rms_current(newCurrent);
                runCurrent = newCurrent;
            }

            else if (load < 1000 / 2)
            {
                uint16_t newCurrent =
                    static_cast<uint16_t>(std::max(static_cast<double>(runCurrent * 0.8), static_cast<double>(100)));
                driver.rms_current(newCurrent);
                runCurrent = newCurrent;
            }

            lastLoadCheckTime = millis();
        }

        static unsigned long lastTempCheckTime = 0;
        if (millis() - lastTempCheckTime >= 200)
        {
            int temp = getTemperature();
            if (temp > 80)
            {
                Logger::getInstance().log(F("⚠️ WARNING: High temperature detected on "));
                Logger::getInstance().log(instanceName);
                Logger::getInstance().log(F(": "));
                Logger::getInstance().logln(String(temp));
                uint16_t reducedCurrent = runCurrent * 0.8;
                driver.rms_current(reducedCurrent);
            }

            lastTempCheckTime = millis();
        }

        if (millis() - lastTempPrintTime >= 1000)
        {
            printTemperature();
            lastTempPrintTime = millis();
        }

        static unsigned long lastMicrostepCheckTime = 0;
        if (millis() - lastMicrostepCheckTime >= 100)
        {
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

uint32_t MotionSystem::TmcController::getDriverStatus()
{
    return driver.DRV_STATUS();
}

void MotionSystem::TmcController::increaseRunCurrent()
{
    if (runCurrent < 1000)
    {
        runCurrent += 100;
        driver.rms_current(runCurrent);
        Logger::getInstance().log(F("Run current increased to: "));
        Logger::getInstance().log(String(runCurrent));
        Logger::getInstance().logln(F("mA (Max: 1000mA)"));
    }

    else
    {
        Logger::getInstance().logln(F("⚠️ Run current at maximum (1000mA)"));
    }
}

void MotionSystem::TmcController::decreaseRunCurrent()
{
    if (runCurrent > 100)
    {
        runCurrent -= 100;
        driver.rms_current(runCurrent);
        Logger::getInstance().log(F("Run current decreased to: "));
        Logger::getInstance().log(String(runCurrent));
        Logger::getInstance().logln(F("mA (Min: 100mA)"));
    }

    else
    {
        Logger::getInstance().logln(F("⚠️ Run current at minimum (100mA)"));
    }
}

void MotionSystem::TmcController::increaseHoldCurrent()
{
    if (holdCurrent < 500)
    {
        holdCurrent += 100;
        driver.ihold(holdCurrent);
        Logger::getInstance().log(F("Hold current increased to: "));
        Logger::getInstance().log(String(holdCurrent));
        Logger::getInstance().logln(F("mA (Max: 500mA)"));
    }

    else
    {
        Logger::getInstance().logln(F("Hold current at maximum (500mA)"));
    }
}

void MotionSystem::TmcController::decreaseHoldCurrent()
{
    if (holdCurrent > 100)
    {
        holdCurrent -= 100;
        driver.ihold(holdCurrent);
        Logger::getInstance().log(F("Hold current decreased to: "));
        Logger::getInstance().log(String(holdCurrent));
        Logger::getInstance().logln(F("mA (Min: 100mA)"));
    }

    else
    {
        Logger::getInstance().logln(F("Hold current at minimum (100mA)"));
    }
}

uint16_t MotionSystem::TmcController::getRunCurrent() const
{
    return runCurrent;
}

uint16_t MotionSystem::TmcController::getHoldCurrent() const
{
    return holdCurrent;
}

void MotionSystem::TmcController::increaseSpeed()
{
    if (speed < 10000)
    {
        speed += 100;
        Logger::getInstance().log(F("Speed increased to: "));
        Logger::getInstance().log(String(speed));
        Logger::getInstance().logln(F(" steps/sec"));
    }

    else
    {
        Logger::getInstance().logln(F("Speed at maximum (10000 steps/sec)"));
    }
}

void MotionSystem::TmcController::decreaseSpeed()
{
    if (speed > 100)
    {
        speed -= 100;
        Logger::getInstance().log(F("Speed decreased to: "));
        Logger::getInstance().log(String(speed));
        Logger::getInstance().logln(F(" steps/sec"));
    }

    else
    {
        Logger::getInstance().logln(F("Speed at minimum (100 steps/sec)"));
    }
}

void MotionSystem::TmcController::increaseAcceleration()
{
    if (acceleration < 10000)
    {
        acceleration += 100;
        driver.AMAX(acceleration);
        Logger::getInstance().log(F("Acceleration increased to: "));
        Logger::getInstance().log(String(acceleration));
        Logger::getInstance().logln(F(" steps/sec²"));
    }

    else
    {
        Logger::getInstance().logln(F("Acceleration at maximum (10000 steps/sec²)"));
    }
}

void MotionSystem::TmcController::decreaseAcceleration()
{
    if (acceleration > 100)
    {
        acceleration -= 100;
        driver.AMAX(acceleration);
        Logger::getInstance().log(F("Acceleration decreased to: "));
        Logger::getInstance().log(String(acceleration));
        Logger::getInstance().logln(F(" steps/sec²"));
    }

    else
    {
        Logger::getInstance().logln(F("Acceleration at minimum (100 steps/sec²)"));
    }
}

uint16_t MotionSystem::TmcController::getSpeed() const
{
    return speed;
}

uint16_t MotionSystem::TmcController::getAcceleration() const
{
    return acceleration;
}

void MotionSystem::TmcController::printDriverStatus()
{
    uint32_t status = driver.DRV_STATUS();
    Logger::getInstance().logln(F("🧠 DRV_STATUS Report"));
    Logger::getInstance().logln(F("──────────────────────────────────────────────"));
    Logger::getInstance().logln(status & (1 << 31) ? F("✅ Standstill (stst)") : F("🌀 Motor moving"));
    Logger::getInstance().logln(status & (1 << 30) ? F("❌ Open load on Phase B (olb)") : F("✅ Phase B OK"));
    Logger::getInstance().logln(status & (1 << 29) ? F("❌ Open load on Phase A (ola)") : F("✅ Phase A OK"));
    Logger::getInstance().logln(status & (1 << 28) ? F("❌ Short to GND on Phase B (s2gb)") : F("✅ Phase B GND OK"));
    Logger::getInstance().logln(status & (1 << 27) ? F("❌ Short to GND on Phase A (s2ga)") : F("✅ Phase A GND OK"));
    Logger::getInstance().logln(status & (1 << 26) ? F("⚠️  Overtemperature pre-warning (otpw)") : F("✅ Temp OK"));
    Logger::getInstance().logln(status & (1 << 25) ? F("🔥 Overtemperature shutdown (ot)") : F("✅ Not overheated"));
    Logger::getInstance().logln(status & (1 << 24) ? F("⚠️  StallGuard: Stall detected!") : F("✅ No stall"));
    Logger::getInstance().logln(status & (1 << 15) ? F("📦 Fullstep active (fsactive)") : F("⏩ Microstepping active"));
    Logger::getInstance().logln(status & (1 << 14) ? F("🎧 StealthChop active (stealth)") : F("⚡ SpreadCycle active"));
    Logger::getInstance().logln(status & (1 << 13) ? F("❌ Short to V+ on Phase B (s2vbs)")
                                                   : F("✅ Phase B Supply OK"));
    Logger::getInstance().logln(status & (1 << 12) ? F("❌ Short to V+ on Phase A (s2vsa)")
                                                   : F("✅ Phase A Supply OK"));
    uint8_t cs_actual = (status >> 17) & 0x0F;
    Logger::getInstance().log(F("CS_ACTUAL (current scaling): "));
    Logger::getInstance().logln(String(cs_actual));
    float current_mA = cs_actual / 32.0 * driver.rms_current();
    Logger::getInstance().log(F("Estimated actual current = "));
    Logger::getInstance().log(String(current_mA));
    Logger::getInstance().logln(F(" mA"));
    uint16_t sg_result = status & 0x03FF;
    if (sg_result < 100)
    {
        Logger::getInstance().logln(F("⚠️  Possi ble stall condition!"));
    }

    else if (sg_result < 500)
    {
        Logger::getInstance().logln(F("ℹ️  Moderate load"));
    }

    else
    {
        Logger::getInstance().logln(F("✅ Light load"));
    }

    Logger::getInstance().logln(F("──────────────────────────────────────────────"));
}

void MotionSystem::TmcController::printDriverConfig()
{
    Logger::getInstance().logln(F("\nDriver Configuration:"));
    Logger::getInstance().logln(F("-------------------"));
    Logger::getInstance().log(F("  Run Current: "));
    Logger::getInstance().log(String(runCurrent));
    Logger::getInstance().logln(F("mA"));
    Logger::getInstance().log(F("  Hold Current: "));
    Logger::getInstance().log(String(holdCurrent));
    Logger::getInstance().logln(F("mA"));
    Logger::getInstance().log(F("  Microsteps: "));
    Logger::getInstance().logln(String(16));
    Logger::getInstance().log(F("  Speed: "));
    Logger::getInstance().log(String(speed));
    Logger::getInstance().logln(F(" steps/sec"));
    Logger::getInstance().log(F("  Acceleration: "));
    Logger::getInstance().log(String(acceleration));
    Logger::getInstance().logln(F(" steps/sec²"));
    Logger::getInstance().logln(F("\nDriver Parameters:"));
    Logger::getInstance().logln(F("------------------"));
    Logger::getInstance().log(F("  GCONF (Global Config): 0x"));
    Logger::getInstance().logDecAsHex(driver.GCONF());
    Logger::getInstance().log(F("  TPOWERDOWN (Power Down Time): "));
    Logger::getInstance().log(String(driver.TPOWERDOWN()));
    Logger::getInstance().logln(F(" tclk"));
    Logger::getInstance().log(F("  TSTEP (Current Step Timing): "));
    Logger::getInstance().log(String(driver.TSTEP()));
    Logger::getInstance().logln(F(" tclk"));
    Logger::getInstance().log(F("  TPWMTHRS (StealthChop Threshold): "));
    Logger::getInstance().log(String(driver.TPWMTHRS()));
    Logger::getInstance().logln(F(" tclk"));
    Logger::getInstance().log(F("  THIGH (Step Pulse High Time): "));
    Logger::getInstance().log(String(driver.THIGH()));
    Logger::getInstance().logln(F(" tclk"));
    Logger::getInstance().log(F("  XDIRECT (Direct Coil Control): 0x"));
    Logger::getInstance().logDecAsHex(driver.XDIRECT());
}

int MotionSystem::TmcController::getTemperature()
{
    uint32_t status  = driver.DRV_STATUS();
    int      rawTemp = (status >> 16) & 0xFF;  // Temperature is in bits 16-23
    return rawTemp;                            // Direct temperature reading in °C (1°C steps)
}

void MotionSystem::TmcController::printTemperature()
{
    int temp = getTemperature();
    if (temp != lastTemperature)
    {
        Logger::getInstance().log(instanceName);
        Logger::getInstance().log(F(": "));
        Logger::getInstance().log(String(temp));
        Logger::getInstance().logln(F("°C"));
        lastTemperature = temp;
    }
}

void MotionSystem::TmcController::toggleStealthChop()
{
    uint32_t currentThreshold = driver.TPWMTHRS();
    if (currentThreshold == 0)
    {
        driver.TPWMTHRS(500);  // Switch to SpreadCycle above 500 steps/sec
        Logger::getInstance().logln(F("⚡ Switched to SpreadCycle mode (more power, more noise)"));
    }

    else
    {
        driver.TPWMTHRS(0);  // Enable StealthChop mode
        Logger::getInstance().logln(F("✅ Switched to StealthChop mode (silent operation)"));
    }
}

void MotionSystem::TmcController::setStealthChopMode(bool enable)
{
    driver.en_pwm_mode(true);  // Ensure StealthChop is available
    if (enable)
    {
        driver.TPWMTHRS(0);  // StealthChop always
        Logger::getInstance().logln(F("✅ StealthChop mode activated (TPWMTHRS = 0)"));
    }

    else
    {
        driver.TPWMTHRS(300);  // SpreadCycle beyond this speed
        Logger::getInstance().logln(F("⚡ SpreadCycle mode activated (TPWMTHRS = 300)"));
    }
}

bool MotionSystem::TmcController::diagnoseTMC5160()
{
    uint32_t status = driver.DRV_STATUS();
    bool     ok     = true;
    if (status & (1 << 27))
    {
        Logger::getInstance().logln(F("❌ ERROR: Short to GND on Phase A (s2ga)"));
        ok = false;
    }

    if (status & (1 << 28))
    {
        Logger::getInstance().logln(F("❌ ERROR: Short to GND on Phase B (s2gb)"));
        ok = false;
    }

    if (status & (1 << 12))
    {
        Logger::getInstance().logln(F("❌ ERROR: Short to supply on Phase A (s2vsa)"));
        ok = false;
    }

    if (status & (1 << 13))
    {
        Logger::getInstance().logln(F("❌ ERROR: Short to supply on Phase B (s2vsb)"));
        ok = false;
    }

    if (status & (1 << 25))
    {
        Logger::getInstance().logln(F("🔥 CRITICAL: Overtemperature shutdown active (ot)"));
        ok = false;
    }

    if (status & (1 << 24))
    {
        Logger::getInstance().logln(F("⚠️  WARNING: Motor stall detected (StallGuard)"));
        ok = false;
    }

    if (ok)
    {
        Logger::getInstance().logln(F("✅ All driver diagnostics OK."));
    }

    return ok;
}

bool MotionSystem::TmcController::testCommunication(bool enableMessage)
{
    if (enableMessage)
    {
        Logger::getInstance().log(instanceName);
        Logger::getInstance().log(F(" - "));
        Logger::getInstance().log(F("Testing SPI communication with TMC5160: "));
    }

    enableSPI();
    uint32_t gconf  = driver.GCONF();
    uint32_t status = driver.DRV_STATUS();
    if (gconf == 0xFFFFFFFF || status == 0xFFFFFFFF)
    {
        if (enableMessage)
        {
            Logger::getInstance().logln(F("❌ Failed\n"));
        }

        return false;
    }

    if (enableMessage)
    {
        Logger::getInstance().logln(F("✅ Ok\n"));
    }

    return true;
}

uint8_t MotionSystem::TmcController::transfer(uint8_t data)
{
    return SPI.transfer(data);
}

void MotionSystem::TmcController::enableDriver(bool enable)
{
    digitalWrite(enPin, enable ? LOW : HIGH);
}

void MotionSystem::TmcController::enableSPI()
{
    digitalWrite(csPin, LOW);
    delay(5);
}

void MotionSystem::TmcController::disableSPI()
{
    digitalWrite(csPin, HIGH);
    delay(5);
}

void MotionSystem::TmcController::resetDriverState()
{
    enableDriver(false);
    enableDriver(true);
}

void MotionSystem::TmcController::setSpreadCycle(bool enable)
{
    spreadCycleEnabled = enable;
    driver.en_pwm_mode(!enable);  // 0 for spread cycle, 1 for stealthChop
}

void MotionSystem::TmcController::setRampMode(uint8_t mode)
{
    rampMode = mode;
    driver.RAMPMODE(mode);
}

void MotionSystem::TmcController::setMaxSpeed(uint32_t speed)
{
    maxSpeed = speed;
    driver.VMAX(speed);
}

void MotionSystem::TmcController::setMaxAcceleration(uint32_t accel)
{
    maxAcceleration = accel;
    driver.a1(accel);
}

void MotionSystem::TmcController::setMaxDeceleration(uint32_t decel)
{
    maxDeceleration = decel;
    driver.d1(decel);
}

void MotionSystem::TmcController::configureDriver2()
{
    driver.begin();
    delay(5);
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Enable internal RSense
    gconf |= (1 << 2);  // Enable StealthChop
    gconf |= (1 << 3);  // Enable microstep interpolation
    gconf |= (1 << 4);  // Double edge step
    gconf |= (1 << 6);  // Enable multi-step filtering
    driver.GCONF(gconf);
    delay(5);
    driver.rms_current(runCurrent);       // RMS current while running
    driver.ihold(holdCurrent);            // Holding current
    driver.irun(runCurrent);              // Running current
    driver.iholddelay(currentHoldDelay);  // Delay to transition to holding current
    driver.TPOWERDOWN(10);                // Motor shutdown time
    driver.microsteps(16);
    driver.intpol(microstepInterpolation);
    driver.TCOOLTHRS(coolStepThreshold);  // CoolStep / StallGuard activation threshold
    driver.semin(5);                      // CoolStep activation (value > 0)
    driver.semax(2);                      // Maximum current increase level
    driver.seup(0b01);                    // Current increase rate
    driver.sedn(0b01);                    // Current decrease rate
    driver.sgt(stallGuardThreshold);      // StallGuard sensitivity
    driver.sfilt(stallGuardFilter);       // Enable pager filter (1 = filter on)
    driver.TPWMTHRS(0);                   // StealthChop always on
    driver.pwm_autoscale(true);           // Enable current auto-tuning
    driver.pwm_autograd(true);            // Enable auto-grading
    driver.pwm_ofs(36);
    driver.pwm_grad(14);
    driver.pwm_freq(1);
    driver.en_pwm_mode(!spreadCycleEnabled);  // true = StealthChop, false = SpreadCycle
    driver.toff(5);                           // Chopper activation
    driver.blank_time(24);
    driver.hysteresis_start(5);
    driver.hysteresis_end(3);
    driver.RAMPMODE(rampMode);
    driver.VSTART(0);       // Start from zero speed
    driver.VMAX(maxSpeed);  // Maximum speed
    driver.VSTOP(10);       // Soft stop, recommended: 5–10
    driver.AMAX(maxAcceleration);
    driver.DMAX(maxDeceleration);
    driver.a1(maxAcceleration);
    driver.v1(maxSpeed / 2);
    driver.d1(maxDeceleration);
    enableDriver(true);
    delay(5);
}

void MotionSystem::TmcController::configureDriver()
{
    driver.begin();
    delay(5);
    uint32_t gconf = 0;
    gconf |= (1 << 0);  // Enable internal RSense
    gconf |= (1 << 2);  // Enable stealthChop
    gconf |= (1 << 3);  // Enable microstep interpolation
    gconf |= (1 << 4);  // Enable double edge step
    gconf |= (1 << 6);  // Enable multistep filtering
    driver.GCONF(gconf);
    delay(5);
    driver.rms_current(runCurrent);
    driver.ihold(holdCurrent);
    driver.irun(runCurrent);
    driver.iholddelay(currentHoldDelay);
    driver.TPOWERDOWN(10);
    driver.microsteps(16);
    driver.intpol(microstepInterpolation);
    driver.TCOOLTHRS(coolStepThreshold);
    driver.sgt(stallGuardThreshold);
    driver.sfilt(stallGuardFilter);
    driver.sgt(stallGuardThreshold);
    driver.TPWMTHRS(0);  // Enable stealthChop by default
    driver.pwm_autoscale(true);
    driver.pwm_autograd(true);
    driver.pwm_ofs(36);
    driver.pwm_grad(14);
    driver.pwm_freq(1);
    driver.en_pwm_mode(!spreadCycleEnabled);  // 0 for spread cycle, 1 for stealthChop
    driver.toff(5);
    driver.blank_time(24);
    driver.hysteresis_start(5);
    driver.hysteresis_end(3);
    driver.RAMPMODE(rampMode);
    driver.VMAX(maxSpeed);
    driver.AMAX(maxAcceleration);
    driver.DMAX(maxDeceleration);
    driver.a1(maxAcceleration);
    driver.v1(maxSpeed / 2);
    driver.d1(maxDeceleration);
    driver.VSTART(0);
    driver.VSTOP(5);
    enableDriver(true);
    delay(5);
}

void MotionSystem::TmcController::setupPins()
{
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    pinMode(csPin, OUTPUT);
    delay(5);
}

void IRAM_ATTR MotionSystem::TmcController::step()
{
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(stepPin, LOW);
    lastStepTime = micros();
    stepCounter  = (stepCounter + 1) % 1000;
}

bool MotionSystem::TmcController::checkAndReinitializeDriver()
{
    uint32_t status = driver.DRV_STATUS();
    if (status == 0 || status == 0xFFFFFFFF)
    {
        enableDriver(false);
        disableSPI();
        enableSPI();
        configureDriver2();
        return true;
    }

    return false;
}

void MotionSystem::TmcController::setDirection(bool forward)
{
    isMoving  = true;
    direction = forward;
    digitalWrite(dirPin, direction ? HIGH : LOW);
    delay(5);
    Logger::getInstance().log(F("Moving "));
    Logger::getInstance().logln(forward ? F("Forward") : F("Reverse"));
}

void MotionSystem::TmcController::printStatusRegister(uint32_t status)
{
    Logger::getInstance().logln(F("\nDriver Status Register:"));
    Logger::getInstance().log(F("Raw Status: 0x"));
    Logger::getInstance().logDecAsHex(status);
    printErrorFlags(status);
    printStallGuardStatus(status);
    printDriverState(status);
}

void MotionSystem::TmcController::printErrorFlags(uint32_t status)
{
    Logger::getInstance().logln(F("\nError Flags:"));
    Logger::getInstance().log(F("  Over Temperature: "));
    Logger::getInstance().logln((status & 0x00000001) ? F("Yes") : F("No"));
    Logger::getInstance().log(F("  Short to Ground A: "));
    Logger::getInstance().logln((status & 0x00000002) ? F("Yes") : F("No"));
    Logger::getInstance().log(F("  Short to Ground B: "));
    Logger::getInstance().logln((status & 0x00000004) ? F("Yes") : F("No"));
    Logger::getInstance().log(F("  Open Load A: "));
    Logger::getInstance().logln((status & 0x00000008) ? F("Yes") : F("No"));
    Logger::getInstance().log(F("  Open Load B: "));
    Logger::getInstance().logln((status & 0x00000010) ? F("Yes") : F("No"));
}

void MotionSystem::TmcController::printStallGuardStatus(uint32_t status)
{
    Logger::getInstance().logln(F("\nStallGuard Status:"));
    Logger::getInstance().log(F("  StallGuard Value: "));
    Logger::getInstance().logln(String((status >> 10) & 0x3FF));
    Logger::getInstance().log(F("  Stall Detected: "));
    Logger::getInstance().logln((status & 0x00000200) ? F("Yes") : F("No"));
}

void MotionSystem::TmcController::printDriverState(uint32_t status)
{
    Logger::getInstance().logln(F("\nDriver State:"));
    Logger::getInstance().log(F("  Standstill: "));
    Logger::getInstance().logln((status & 0x00000400) ? F("Yes") : F("No"));
    Logger::getInstance().log(F("  Velocity Reached: "));
    Logger::getInstance().logln((status & 0x00000800) ? F("Yes") : F("No"));
    Logger::getInstance().log(F("  Position Reached: "));
    Logger::getInstance().logln((status & 0x00001000) ? F("Yes") : F("No"));
}

uint32_t MotionSystem::TmcController::calculateStepInterval(Types::Speed speed)
{
    if (abs(speed) < 1)
        return 0;                 // Prevent division by zero
    return 1000000 / abs(speed);  // Convert Hz to microseconds
}

MotionSystem::Types::StepPosition MotionSystem::TmcController::micronsToSteps(
    MotionSystem::Types::MicronPosition microns)
{
    return roundf(microns * Config::System::MOTOR_STEPS_PER_MICRON);
}

MotionSystem::Types::StepPosition MotionSystem::TmcController::pixelsToSteps(MotionSystem::Types::PixelPosition pixels)
{
    return micronsToSteps(pixels * Config::System::PIXEL_SIZE);
}