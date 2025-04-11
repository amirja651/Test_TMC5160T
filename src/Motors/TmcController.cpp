#include "Motors/TmcController.h"
#include "Helper/Logger.h"
#include "Helper/System.h"

namespace MotionSystem
{
    TmcController::TmcController(const char* name, uint8_t csPin, uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                                 MotorType motorType, uint8_t mosiPin, uint8_t misoPin, uint8_t sckPin)
        : driver(csPin, mosiPin, misoPin, sckPin),
          instanceName(name),
          csPin(csPin),
          stepPin(stepPin),
          dirPin(dirPin),
          enPin(enPin),
          mosiPin(mosiPin),
          misoPin(misoPin),
          sckPin(sckPin),
          motorType(motorType),
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
        // Set motor-specific default values
        switch (motorType)
        {
            case MotorType::NEMA11_HS13_1004H:
                runCurrent          = 500;  // 1.0A
                holdCurrent         = 500;   // 0.5A
                maxSpeed            = 1000;
                maxAcceleration     = 10000;
                maxDeceleration     = 10000;
                coolStepThreshold   = 1500;
                stallGuardThreshold = 15;
                break;

            case MotorType::P28SHD4611_12SK:
                runCurrent          = 100;  // 0.5A
                holdCurrent         = 100;  // 0.25A
                maxSpeed            = 200;
                maxAcceleration     = 500;
                maxDeceleration     = 500;
                coolStepThreshold   = 1000;
                stallGuardThreshold = 10;
                break;

            default:
                Serial.println(F("❌ Unknown motor type."));
                return;
        }
    }

    void TmcController::begin()
    {
        if (motorType == MotorType::UNKNOWN)
        {
            return;
        }

        setupPins();
        disableSPI();
        resetDriverState();
        configureDriver();
    }

    void TmcController::moveForward()
    {
        if (!diagnoseTMC5160())
        {
            Serial.println(F("❌ Driver error detected. Motor will not move."));
            return;
        }

        setDirection(true);
    }

    void TmcController::moveReverse()
    {
        if (!diagnoseTMC5160())
        {
            Serial.println(F("❌ Driver error detected. Motor will not move."));
            return;
        }

        setDirection(false);
    }

    void TmcController::stop()
    {
        isMoving = false;
        driver.ihold(100);  // Reduce to ultra low hold current
    }

    void TmcController::update()
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
                    Serial.print(F("Diagnostics - "));
                    Serial.print(instanceName);
                    Serial.print(F(": Load="));
                    Serial.print(String(load_value));
                    Serial.print(F(", Temp="));
                    Serial.print(String(temp));
                    Serial.println(F("°C"));
                    uint32_t status = driver.DRV_STATUS();
                    if (status & 0x00000200)
                    {
                        Serial.println(F("❌ HARD STALL detected! Stopping motor."));
                        stop();  // Stop motor on stall
                        printStallGuardStatus(status);
                    }

                    if (driver.sg_result() < stallGuardThreshold)
                    {
                        Serial.print(F("⚠️ Pre-stall warning on "));
                        Serial.println(instanceName);
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
                    Serial.print(F("⚠️ WARNING: High load detected on "));
                    Serial.print(instanceName);
                    Serial.print(F(": "));
                    Serial.println(String(load));
                }

                else if (load > 1000)
                {
                    uint16_t newCurrent = static_cast<uint16_t>(
                        std::min(static_cast<double>(runCurrent * 1.2), static_cast<double>(1000)));
                    driver.rms_current(newCurrent);
                    runCurrent = newCurrent;
                }

                else if (load < 1000 / 2)
                {
                    uint16_t newCurrent = static_cast<uint16_t>(
                        std::max(static_cast<double>(runCurrent * 0.8), static_cast<double>(100)));
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
                    Serial.print(F("⚠️ WARNING: High temperature detected on "));
                    Serial.print(instanceName);
                    Serial.print(F(": "));
                    Serial.println(String(temp));
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

    uint32_t TmcController::getDriverStatus()
    {
        return driver.DRV_STATUS();
    }

    void TmcController::increaseRunCurrent()
    {
        if (runCurrent < 1000)
        {
            runCurrent += 100;
            driver.rms_current(runCurrent);
            Serial.print(F("Run current increased to: "));
            Serial.print(String(runCurrent));
            Serial.println(F("mA (Max: 1000mA)"));
        }

        else
        {
            Serial.println(F("⚠️ Run current at maximum (1000mA)"));
        }
    }

    void TmcController::decreaseRunCurrent()
    {
        if (runCurrent > 100)
        {
            runCurrent -= 100;
            driver.rms_current(runCurrent);
            Serial.print(F("Run current decreased to: "));
            Serial.print(String(runCurrent));
            Serial.println(F("mA (Min: 100mA)"));
        }

        else
        {
            Serial.println(F("⚠️ Run current at minimum (100mA)"));
        }
    }

    void TmcController::increaseHoldCurrent()
    {
        if (holdCurrent < 500)
        {
            holdCurrent += 100;
            driver.ihold(holdCurrent);
            Serial.print(F("Hold current increased to: "));
            Serial.print(String(holdCurrent));
            Serial.println(F("mA (Max: 500mA)"));
        }

        else
        {
            Serial.println(F("Hold current at maximum (500mA)"));
        }
    }

    void TmcController::decreaseHoldCurrent()
    {
        if (holdCurrent > 100)
        {
            holdCurrent -= 100;
            driver.ihold(holdCurrent);
            Serial.print(F("Hold current decreased to: "));
            Serial.print(String(holdCurrent));
            Serial.println(F("mA (Min: 100mA)"));
        }

        else
        {
            Serial.println(F("Hold current at minimum (100mA)"));
        }
    }

    uint16_t TmcController::getRunCurrent() const
    {
        return runCurrent;
    }

    uint16_t TmcController::getHoldCurrent() const
    {
        return holdCurrent;
    }

    void TmcController::increaseSpeed()
    {
        if (speed < 10000)
        {
            speed += 100;
            Serial.print(F("Speed increased to: "));
            Serial.print(String(speed));
            Serial.println(F(" steps/sec"));
        }

        else
        {
            Serial.println(F("Speed at maximum (10000 steps/sec)"));
        }
    }

    void TmcController::decreaseSpeed()
    {
        if (speed > 100)
        {
            speed -= 100;
            Serial.print(F("Speed decreased to: "));
            Serial.print(String(speed));
            Serial.println(F(" steps/sec"));
        }

        else
        {
            Serial.println(F("Speed at minimum (100 steps/sec)"));
        }
    }

    void TmcController::increaseAcceleration()
    {
        if (acceleration < 10000)
        {
            acceleration += 100;
            driver.AMAX(acceleration);
            Serial.print(F("Acceleration increased to: "));
            Serial.print(String(acceleration));
            Serial.println(F(" steps/sec²"));
        }

        else
        {
            Serial.println(F("Acceleration at maximum (10000 steps/sec²)"));
        }
    }

    void TmcController::decreaseAcceleration()
    {
        if (acceleration > 100)
        {
            acceleration -= 100;
            driver.AMAX(acceleration);
            Serial.print(F("Acceleration decreased to: "));
            Serial.print(String(acceleration));
            Serial.println(F(" steps/sec²"));
        }

        else
        {
            Serial.println(F("Acceleration at minimum (100 steps/sec²)"));
        }
    }

    uint16_t TmcController::getSpeed() const
    {
        return speed;
    }

    uint16_t TmcController::getAcceleration() const
    {
        return acceleration;
    }

    void TmcController::printDriverStatus()
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
        Serial.println(String(cs_actual));
        float current_mA = cs_actual / 32.0 * driver.rms_current();
        Serial.print(F("Estimated actual current = "));
        Serial.print(String(current_mA));
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

    void TmcController::printDriverConfig()
    {
        Serial.println(F("\nDriver Configuration:"));
        Serial.println(F("-------------------"));
        Serial.print(F("  Run Current: "));
        Serial.print(String(runCurrent));
        Serial.println(F("mA"));
        Serial.print(F("  Hold Current: "));
        Serial.print(String(holdCurrent));
        Serial.println(F("mA"));
        Serial.print(F("  Microsteps: "));
        Serial.println(String(16));
        Serial.print(F("  Speed: "));
        Serial.print(String(speed));
        Serial.println(F(" steps/sec"));
        Serial.print(F("  Acceleration: "));
        Serial.print(String(acceleration));
        Serial.println(F(" steps/sec²"));
        Serial.println(F("\nDriver Parameters:"));
        Serial.println(F("------------------"));
        Serial.print(F("  GCONF (Global Config): 0x"));
        Serial.print(driver.GCONF(), HEX);
        Serial.print(F("  TPOWERDOWN (Power Down Time): "));
        Serial.print(String(driver.TPOWERDOWN()));
        Serial.println(F(" tclk"));
        Serial.print(F("  TSTEP (Current Step Timing): "));
        Serial.print(String(driver.TSTEP()));
        Serial.println(F(" tclk"));
        Serial.print(F("  TPWMTHRS (StealthChop Threshold): "));
        Serial.print(String(driver.TPWMTHRS()));
        Serial.println(F(" tclk"));
        Serial.print(F("  THIGH (Step Pulse High Time): "));
        Serial.print(String(driver.THIGH()));
        Serial.println(F(" tclk"));
        Serial.print(F("  XDIRECT (Direct Coil Control): 0x"));
        Serial.print(driver.XDIRECT(), HEX);
    }

    int TmcController::getTemperature()
    {
        uint32_t status  = driver.DRV_STATUS();
        int      rawTemp = (status >> 16) & 0xFF;  // Temperature is in bits 16-23
        return rawTemp;                            // Direct temperature reading in °C (1°C steps)
    }

    void TmcController::printTemperature()
    {
        int temp = getTemperature();
        if (temp != lastTemperature)
        {
            Serial.print(instanceName);
            Serial.print(F(": "));
            Serial.print(String(temp));
            Serial.println(F("°C"));
            lastTemperature = temp;
        }
    }

    void TmcController::toggleStealthChop()
    {
        uint32_t currentThreshold = driver.TPWMTHRS();
        if (currentThreshold == 0)
        {
            driver.TPWMTHRS(500);  // Switch to SpreadCycle above 500 steps/sec
            Serial.println(F("⚡ Switched to SpreadCycle mode (more power, more noise)"));
        }

        else
        {
            driver.TPWMTHRS(0);  // Enable StealthChop mode
            Serial.println(F("✅ Switched to StealthChop mode (silent operation)"));
        }
    }

    void TmcController::setStealthChopMode(bool enable)
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

    bool TmcController::diagnoseTMC5160()
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

    bool TmcController::testCommunication(bool enableMessage)
    {
        if (enableMessage)
        {
            Serial.print(instanceName);
            Serial.print(F(" - "));
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

    uint8_t TmcController::transfer(uint8_t data)
    {
        return SPI.transfer(data);
    }

    void TmcController::enableDriver(bool enable)
    {
        digitalWrite(enPin, enable ? LOW : HIGH);
    }

    void TmcController::enableSPI()
    {
        digitalWrite(csPin, LOW);
        delay(5);
    }

    void TmcController::disableSPI()
    {
        digitalWrite(csPin, HIGH);
        delay(5);
    }

    void TmcController::resetDriverState()
    {
        enableDriver(false);
        enableDriver(true);
    }

    void TmcController::setSpreadCycle(bool enable)
    {
        spreadCycleEnabled = enable;
        driver.en_pwm_mode(!enable);  // 0 for spread cycle, 1 for stealthChop
    }

    void TmcController::setRampMode(uint8_t mode)
    {
        rampMode = mode;
        driver.RAMPMODE(mode);
    }

    void TmcController::setMaxSpeed(uint32_t speed)
    {
        maxSpeed = speed;
        driver.VMAX(speed);
    }

    void TmcController::setMaxAcceleration(uint32_t accel)
    {
        maxAcceleration = accel;
        driver.a1(accel);
    }

    void TmcController::setMaxDeceleration(uint32_t decel)
    {
        maxDeceleration = decel;
        driver.d1(decel);
    }

    void TmcController::configureDriver()
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

        // Motor-specific configuration
        switch (motorType)
        {
            case MotorType::NEMA11_HS13_1004H:
                // Configure for NEMA11 motor
                driver.rms_current(runCurrent);       // 1.0A RMS current
                driver.ihold(holdCurrent);            // 0.5A holding current
                driver.irun(runCurrent);              // 1.0A running current
                driver.iholddelay(currentHoldDelay);  // Delay to transition to holding current
                driver.TPOWERDOWN(10);                // Motor shutdown time
                driver.microsteps(16);                // 16 microsteps for smooth operation
                driver.intpol(microstepInterpolation);
                driver.TCOOLTHRS(coolStepThreshold);  // CoolStep activation threshold
                driver.semin(5);                      // CoolStep activation
                driver.semax(2);                      // Maximum current increase level
                driver.seup(0b01);                    // Current increase rate
                driver.sedn(0b01);                    // Current decrease rate
                driver.sgt(stallGuardThreshold);      // StallGuard sensitivity
                driver.sfilt(stallGuardFilter);       // Enable pager filter
                driver.TPWMTHRS(0);                   // StealthChop always on
                driver.pwm_autoscale(true);           // Enable current auto-tuning
                driver.pwm_autograd(true);            // Enable auto-grading
                driver.pwm_ofs(36);
                driver.pwm_grad(14);
                driver.pwm_freq(1);
                driver.en_pwm_mode(!spreadCycleEnabled);
                driver.toff(5);  // Chopper activation
                driver.blank_time(24);
                driver.hysteresis_start(5);
                driver.hysteresis_end(3);
                driver.RAMPMODE(rampMode);
                driver.VSTART(0);       // Start from zero speed
                driver.VMAX(maxSpeed);  // Maximum speed
                driver.VSTOP(10);       // Soft stop
                driver.AMAX(maxAcceleration);
                driver.DMAX(maxDeceleration);
                driver.a1(maxAcceleration);
                driver.v1(maxSpeed / 2);
                driver.d1(maxDeceleration);
                break;

            case MotorType::P28SHD4611_12SK:
                // Configure for P28SHD4611 motor
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
                break;

            default:
                Serial.println(F("❌ Unknown motor type."));
                return;
        }

        enableDriver(true);
        delay(5);
    }

    void TmcController::setupPins()
    {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);
        pinMode(enPin, OUTPUT);
        pinMode(csPin, OUTPUT);
        delay(5);
    }

    void IRAM_ATTR TmcController::step()
    {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(stepPin, LOW);
        lastStepTime = micros();
        stepCounter  = (stepCounter + 1) % 1000;
    }

    bool TmcController::checkAndReinitializeDriver()
    {
        uint32_t status = driver.DRV_STATUS();
        if (status == 0 || status == 0xFFFFFFFF)
        {
            enableDriver(false);
            disableSPI();
            enableSPI();
            configureDriver();
            return true;
        }

        return false;
    }

    void TmcController::setDirection(bool forward)
    {
        isMoving  = true;
        direction = forward;
        digitalWrite(dirPin, direction ? HIGH : LOW);
        delay(5);
        Serial.print(instanceName);
        Serial.print(F(" - "));
        Serial.print(F("Direction "));
        Serial.println(forward ? F("Forward") : F("Reverse"));
    }

    void TmcController::printStatusRegister(uint32_t status)
    {
        Serial.println(F("\nDriver Status Register:"));
        Serial.print(F("Raw Status: 0x"));
        Serial.print(status, HEX);
        printErrorFlags(status);
        printStallGuardStatus(status);
        printDriverState(status);
    }

    void TmcController::printErrorFlags(uint32_t status)
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

    void TmcController::printStallGuardStatus(uint32_t status)
    {
        Serial.println(F("\nStallGuard Status:"));
        Serial.print(F("  StallGuard Value: "));
        Serial.println(String((status >> 10) & 0x3FF));
        Serial.print(F("  Stall Detected: "));
        Serial.println((status & 0x00000200) ? F("Yes") : F("No"));
    }

    void TmcController::printDriverState(uint32_t status)
    {
        Serial.println(F("\nDriver State:"));
        Serial.print(F("  Standstill: "));
        Serial.println((status & 0x00000400) ? F("Yes") : F("No"));
        Serial.print(F("  Velocity Reached: "));
        Serial.println((status & 0x00000800) ? F("Yes") : F("No"));
        Serial.print(F("  Position Reached: "));
        Serial.println((status & 0x00001000) ? F("Yes") : F("No"));
    }

    uint32_t TmcController::calculateStepInterval(Types::Speed speed)
    {
        if (abs(speed) < 1)
            return 0;                 // Prevent division by zero
        return 1000000 / abs(speed);  // Convert Hz to microseconds
    }

    Types::StepPosition TmcController::micronsToSteps(Types::MicronPosition microns)
    {
        return roundf(microns * System::MOTOR_STEPS_PER_MICRON);
    }

    Types::StepPosition TmcController::pixelsToSteps(Types::PixelPosition pixels)
    {
        return micronsToSteps(pixels * System::PIXEL_SIZE);
    }

    String TmcController::getInstanceName()
    {
        return instanceName;
    }
}  // namespace MotionSystem