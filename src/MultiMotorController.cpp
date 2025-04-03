#include "MultiMotorController.h"
#include "config.h"

// Singleton instance getter
MultiMotorController& MultiMotorController::getInstance() {
    static MultiMotorController instance;
    return instance;
}

// Private constructor for singleton pattern
MultiMotorController::MultiMotorController()
    : numMotors(1),  // Default to 1 motor
      commonPins(new uint8_t[3]{Config::SPI::MOSI, Config::SPI::MISO, Config::SPI::SCK}),
      motorPins(new uint8_t[16]{
          Config::TMC5160T_Driver::D1_CS, Config::TMC5160T_Driver::D1_STEP_PIN, Config::TMC5160T_Driver::D1_DIR_PIN,
          Config::TMC5160T_Driver::D1_EN_PIN, Config::TMC5160T_Driver::D2_CS, Config::TMC5160T_Driver::D2_STEP_PIN,
          Config::TMC5160T_Driver::D2_DIR_PIN, Config::TMC5160T_Driver::D2_EN_PIN, Config::TMC5160T_Driver::D3_CS,
          Config::TMC5160T_Driver::D3_STEP_PIN, Config::TMC5160T_Driver::D3_DIR_PIN, Config::TMC5160T_Driver::D3_EN_PIN,
          Config::TMC5160T_Driver::D4_CS, Config::TMC5160T_Driver::D4_STEP_PIN, Config::TMC5160T_Driver::D4_DIR_PIN,
          Config::TMC5160T_Driver::D4_EN_PIN}) {
    // Initialize SPI interface
    SPI.begin(commonPins[0], commonPins[1], commonPins[2]);
    SPI.setFrequency(1000000);
    SPI.setHwCs(false);

    // Create drivers and motor states
    drivers.reserve(numMotors);
    motors.reserve(numMotors);

    // Create driver instance with proper initialization
    drivers.emplace_back(motorPins[0], 0.11f, commonPins[0], commonPins[1], commonPins[2]);

    // Create motor state
    motors.emplace_back(drivers.back(), motorPins[0], motorPins[1], motorPins[2], motorPins[3]);
}

MultiMotorController::MultiMotorController(uint8_t numMotors, const uint8_t* commonPins, const uint8_t* motorPins)
    : numMotors(numMotors), commonPins(commonPins), motorPins(motorPins) {
    // Initialize SPI interface
    SPI.begin(commonPins[0], commonPins[1], commonPins[2]);
    SPI.setFrequency(1000000);
    SPI.setHwCs(false);

    // Create drivers and motor states
    drivers.reserve(numMotors);
    motors.reserve(numMotors);

    for (uint8_t i = 0; i < numMotors; ++i) {
        // Get motor-specific pins
        uint8_t csPin   = motorPins[i * 4];
        uint8_t stepPin = motorPins[i * 4 + 1];
        uint8_t dirPin  = motorPins[i * 4 + 2];
        uint8_t enPin   = motorPins[i * 4 + 3];

        // Create driver instance with proper initialization
        drivers.emplace_back(csPin, 0.11f, commonPins[0], commonPins[1], commonPins[2]);

        // Create motor state
        motors.emplace_back(drivers.back(), csPin, stepPin, dirPin, enPin);
    }
}

void MultiMotorController::begin() {
    for (auto& motor : motors) {
        motor.begin();
    }
}

void MultiMotorController::update() {
    for (auto& motor : motors) {
        motor.update();
    }
}

void MultiMotorController::moveForward(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).moveForward();
    }
}

void MultiMotorController::moveReverse(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).moveReverse();
    }
}

void MultiMotorController::stop(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).stop();
    }
}

void MultiMotorController::increaseRunCurrent(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).increaseRunCurrent();
    }
}

void MultiMotorController::decreaseRunCurrent(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).decreaseRunCurrent();
    }
}

void MultiMotorController::increaseHoldCurrent(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).increaseHoldCurrent();
    }
}

void MultiMotorController::decreaseHoldCurrent(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).decreaseHoldCurrent();
    }
}

uint16_t MultiMotorController::getRunCurrent(uint8_t motorIndex) const {
    return isValidMotorIndex(motorIndex) ? getMotorState(motorIndex).getRunCurrent() : 0;
}

uint16_t MultiMotorController::getHoldCurrent(uint8_t motorIndex) const {
    return isValidMotorIndex(motorIndex) ? getMotorState(motorIndex).getHoldCurrent() : 0;
}

void MultiMotorController::increaseSpeed(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).increaseSpeed();
    }
}

void MultiMotorController::decreaseSpeed(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).decreaseSpeed();
    }
}

void MultiMotorController::increaseAcceleration(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).increaseAcceleration();
    }
}

void MultiMotorController::decreaseAcceleration(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).decreaseAcceleration();
    }
}

uint16_t MultiMotorController::getSpeed(uint8_t motorIndex) const {
    return isValidMotorIndex(motorIndex) ? getMotorState(motorIndex).getSpeed() : 0;
}

uint16_t MultiMotorController::getAcceleration(uint8_t motorIndex) const {
    return isValidMotorIndex(motorIndex) ? getMotorState(motorIndex).getAcceleration() : 0;
}

uint32_t MultiMotorController::getDriverStatus(uint8_t motorIndex) const {
    return isValidMotorIndex(motorIndex) ? getMotorState(motorIndex).getDriverStatus() : 0;
}

int MultiMotorController::getTemperature(uint8_t motorIndex) const {
    return isValidMotorIndex(motorIndex) ? getMotorState(motorIndex).getTemperature() : 0;
}

void MultiMotorController::toggleStealthChop(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        getMotorState(motorIndex).toggleStealthChop();
    }
}

void MultiMotorController::printDriverStatus(uint8_t motorIndex) const {
    if (isValidMotorIndex(motorIndex)) {
        uint32_t status = getMotorState(motorIndex).getDriverStatus();
        Serial.print("Motor ");
        Serial.println(motorIndex);
        getMotorState(motorIndex).printStatusRegister(status);
        getMotorState(motorIndex).printErrorFlags(status);
        getMotorState(motorIndex).printStallGuardStatus(status);
        getMotorState(motorIndex).printDriverState(status);
    }
}

void MultiMotorController::printDriverConfig(uint8_t motorIndex) const {
    if (isValidMotorIndex(motorIndex)) {
        Serial.print("Motor ");
        Serial.println(motorIndex);
        Serial.print("Run Current: ");
        Serial.println(getRunCurrent(motorIndex));
        Serial.print("Hold Current: ");
        Serial.println(getHoldCurrent(motorIndex));
        Serial.print("Speed: ");
        Serial.println(getSpeed(motorIndex));
        Serial.print("Acceleration: ");
        Serial.println(getAcceleration(motorIndex));
    }
}

void MultiMotorController::printTemperature(uint8_t motorIndex) const {
    if (isValidMotorIndex(motorIndex)) {
        Serial.print("Motor ");
        Serial.print(motorIndex);
        Serial.print(" Temperature: ");
        Serial.println(getTemperature(motorIndex));
    }
}

bool MultiMotorController::isValidMotorIndex(uint8_t motorIndex) const {
    return motorIndex < numMotors;
}

MotorState& MultiMotorController::getMotorState(uint8_t motorIndex) {
    return motors[motorIndex];
}

const MotorState& MultiMotorController::getMotorState(uint8_t motorIndex) const {
    return motors[motorIndex];
}

void MultiMotorController::printErrorFlags(uint32_t status) const {
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

void MultiMotorController::printStallGuardStatus(uint32_t status) const {
    Serial.print("StallGuard Status: ");
    Serial.println((status >> 8) & 0xFF);
}

void MultiMotorController::printDriverState(uint32_t status) const {
    Serial.print("Driver State: ");
    Serial.println((status >> 16) & 0xFF);
}

void MultiMotorController::testCommunication() {
    Serial.println("Testing SPI communication...");
    for (uint8_t i = 0; i < numMotors; ++i) {
        selectDevice(i);
        transfer(Config::SPI::TEST_VALUE);
        deselectDevice(i);
    }
    Serial.println("SPI test complete");
}

void MultiMotorController::selectDevice(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        digitalWrite(motorPins[motorIndex * 4], LOW);  // CS pin is the first pin for each motor
    }
}

void MultiMotorController::deselectDevice(uint8_t motorIndex) {
    if (isValidMotorIndex(motorIndex)) {
        digitalWrite(motorPins[motorIndex * 4], HIGH);  // CS pin is the first pin for each motor
    }
}

uint8_t MultiMotorController::transfer(uint8_t data) {
    return SPI.transfer(data);
}

void MultiMotorController::resetDriver(uint8_t motorIndex) {
    if (motorIndex < numMotors) {
        drivers[motorIndex].reset();
    }
}