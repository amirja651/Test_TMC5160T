#include "MotorState.h"
#include "config.h"

MotorState::MotorState(TMC5160Stepper& driver, uint8_t csPin, uint8_t stepPin, uint8_t dirPin, uint8_t enPin)
    : driver(driver),
      csPin(csPin),
      stepPin(stepPin),
      dirPin(dirPin),
      enPin(enPin),
      moving(false),
      direction(true),
      lastStepTime(0),
      stepCounter(0),
      runCurrent(Config::MotorSpecs::Operation::RUN_CURRENT),
      holdCurrent(Config::MotorSpecs::Operation::HOLD_CURRENT),
      speed(Config::MotorSpecs::Operation::SPEED),
      acceleration(Config::MotorSpecs::Operation::ACCELERATION),
      lastTempPrintTime(0),
      lastTemperature(0) {}

void MotorState::begin() {
    setupPins();
    configureDriver();
}

void MotorState::update() {
    if (moving) {
        unsigned long currentTime = micros();
        if (currentTime - lastStepTime >= (1000000UL / speed)) {
            step();
            lastStepTime = currentTime;
        }
    }
    checkStall();
}

void MotorState::moveForward() {
    digitalWrite(dirPin, HIGH);
    direction = true;
    moving    = true;
}

void MotorState::moveReverse() {
    digitalWrite(dirPin, LOW);
    direction = false;
    moving    = true;
}

void MotorState::stop() {
    moving = false;
}

void MotorState::step() {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(Config::MotorController::STEP_DELAY);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(Config::MotorController::STEP_DELAY);
    stepCounter++;
}

void MotorState::increaseRunCurrent() {
    if (runCurrent < Config::TMC5160T_Driver::MAX_CURRENT_MA) {
        runCurrent += Config::TMC5160T_Driver::CURRENT_STEP_MA;
        driver.rms_current(runCurrent);
    }
}

void MotorState::decreaseRunCurrent() {
    if (runCurrent > Config::TMC5160T_Driver::MIN_CURRENT_MA) {
        runCurrent -= Config::TMC5160T_Driver::CURRENT_STEP_MA;
        driver.rms_current(runCurrent);
    }
}

void MotorState::increaseHoldCurrent() {
    if (holdCurrent < Config::TMC5160T_Driver::MAX_CURRENT_MA) {
        holdCurrent += Config::TMC5160T_Driver::CURRENT_STEP_MA;
        driver.ihold(holdCurrent);
    }
}

void MotorState::decreaseHoldCurrent() {
    if (holdCurrent > Config::TMC5160T_Driver::MIN_CURRENT_MA) {
        holdCurrent -= Config::TMC5160T_Driver::CURRENT_STEP_MA;
        driver.ihold(holdCurrent);
    }
}

uint16_t MotorState::getRunCurrent() const {
    return runCurrent;
}

uint16_t MotorState::getHoldCurrent() const {
    return holdCurrent;
}

void MotorState::increaseSpeed() {
    if (speed < Config::TMC5160T_Driver::MAX_SPEED) {
        speed += Config::TMC5160T_Driver::SPEED_STEP;
        driver.VMAX(speed);
    }
}

void MotorState::decreaseSpeed() {
    if (speed > Config::TMC5160T_Driver::MIN_SPEED) {
        speed -= Config::TMC5160T_Driver::SPEED_STEP;
        driver.VMAX(speed);
    }
}

void MotorState::increaseAcceleration() {
    if (acceleration < Config::MotorController::MAX_ACCEL) {
        acceleration += Config::MotorController::ACCEL_STEP;
        driver.AMAX(acceleration);
    }
}

void MotorState::decreaseAcceleration() {
    if (acceleration > Config::MotorController::MIN_ACCEL) {
        acceleration -= Config::MotorController::ACCEL_STEP;
        driver.AMAX(acceleration);
    }
}

uint16_t MotorState::getSpeed() const {
    return speed;
}

uint16_t MotorState::getAcceleration() const {
    return acceleration;
}

uint32_t MotorState::getDriverStatus() const {
    return driver.DRV_STATUS();
}

int MotorState::getTemperature() const {
    uint32_t status  = driver.DRV_STATUS();
    int      rawTemp = (status >> 16) & 0xFF;  // Temperature is in bits 16-23
    return (rawTemp - 1) * 1.5;                // Convert to Celsius
}

void MotorState::toggleStealthChop() {
    uint32_t currentThreshold = driver.TPWMTHRS();
    if (currentThreshold == 0) {
        driver.TPWMTHRS(500);  // Switch to SpreadCycle
    } else {
        driver.TPWMTHRS(0);  // Switch to StealthChop
    }
}

bool MotorState::isMoving() const {
    return moving;
}

bool MotorState::getDirection() const {
    return direction;
}

void MotorState::configureDriver() {
    driver.begin();
    driver.toff(Config::TMC5160T_Driver::TOFF);
    driver.blank_time(Config::TMC5160T_Driver::BLANK_TIME);
    driver.rms_current(runCurrent);
    driver.microsteps(Config::TMC5160T_Driver::MICROSTEPS);
    driver.TCOOLTHRS(Config::TMC5160T_Driver::TCOOLTHRS);
    driver.TPOWERDOWN(Config::TMC5160T_Driver::TPOWERDOWN);
    driver.VMAX(speed);
    driver.AMAX(acceleration);
}

void MotorState::setupPins() {
    pinMode(csPin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(enPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, HIGH);
    digitalWrite(enPin, LOW);
}

bool MotorState::checkAndReinitializeDriver() {
    uint32_t status = driver.DRV_STATUS();
    if (status == Config::MotorController::INVALID_STATUS) {
        configureDriver();
        return true;
    }
    return false;
}

void MotorState::handlePowerLoss() {
    stop();
    configureDriver();
}

void MotorState::checkStall() {
    uint32_t status = driver.DRV_STATUS();
    if (status & Config::TMC5160T_Driver::STALL_BIT_MASK) {
        stop();
    }
}

void MotorState::printStatusRegister(uint32_t status) const {
    Serial.print("Status Register: 0x");
    Serial.println(status, HEX);
}

void MotorState::printErrorFlags(uint32_t status) const {
    Serial.print("Error Flags: 0x");
    Serial.println(status & 0xFF, HEX);
}

void MotorState::printStallGuardStatus(uint32_t status) const {
    Serial.print("StallGuard Status: ");
    Serial.println((status >> 8) & 0xFF);
}

void MotorState::printDriverState(uint32_t status) const {
    Serial.print("Driver State: ");
    Serial.println((status >> 16) & 0xFF);
}