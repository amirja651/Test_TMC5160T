#ifndef MOTOR_STATE_H
#define MOTOR_STATE_H

#include <Arduino.h>
#include <TMCStepper.h>

/**
 * @brief Encapsulates the state and operations of a single TMC5160 motor
 */
class MotorState {
public:
    /**
     * @brief Constructor for MotorState
     * @param driver TMC5160Stepper driver instance
     * @param csPin Chip select pin for this motor
     * @param stepPin Step pin for this motor
     * @param dirPin Direction pin for this motor
     * @param enPin Enable pin for this motor
     */
    MotorState(TMC5160Stepper& driver, uint8_t csPin, uint8_t stepPin, uint8_t dirPin, uint8_t enPin);

    // Move constructor
    MotorState(MotorState&& other) noexcept
        : driver(other.driver),
          csPin(other.csPin),
          stepPin(other.stepPin),
          dirPin(other.dirPin),
          enPin(other.enPin),
          moving(other.moving),
          direction(other.direction),
          lastStepTime(other.lastStepTime),
          stepCounter(other.stepCounter),
          runCurrent(other.runCurrent),
          holdCurrent(other.holdCurrent),
          speed(other.speed),
          acceleration(other.acceleration),
          lastTempPrintTime(other.lastTempPrintTime),
          lastTemperature(other.lastTemperature) {}

    // Move assignment operator
    MotorState& operator=(MotorState&& other) noexcept {
        if (this != &other) {
            // Reference members can't be reassigned, so we don't modify them
            moving            = other.moving;
            direction         = other.direction;
            lastStepTime      = other.lastStepTime;
            stepCounter       = other.stepCounter;
            runCurrent        = other.runCurrent;
            holdCurrent       = other.holdCurrent;
            speed             = other.speed;
            acceleration      = other.acceleration;
            lastTempPrintTime = other.lastTempPrintTime;
            lastTemperature   = other.lastTemperature;
        }
        return *this;
    }

    // Motor control methods
    void begin();
    void update();
    void moveForward();
    void moveReverse();
    void stop();
    void step();

    // Current control methods
    void     increaseRunCurrent();
    void     decreaseRunCurrent();
    void     increaseHoldCurrent();
    void     decreaseHoldCurrent();
    uint16_t getRunCurrent() const;
    uint16_t getHoldCurrent() const;

    // Speed and acceleration control methods
    void     increaseSpeed();
    void     decreaseSpeed();
    void     increaseAcceleration();
    void     decreaseAcceleration();
    uint16_t getSpeed() const;
    uint16_t getAcceleration() const;

    // Status and monitoring methods
    uint32_t getDriverStatus() const;
    int      getTemperature() const;
    void     toggleStealthChop();
    bool     isMoving() const;
    bool     getDirection() const;

    // Status printing methods
    void printStatusRegister(uint32_t status) const;
    void printErrorFlags(uint32_t status) const;
    void printStallGuardStatus(uint32_t status) const;
    void printDriverState(uint32_t status) const;

private:
    // Delete copy constructor and assignment operator
    MotorState(const MotorState&)            = delete;
    MotorState& operator=(const MotorState&) = delete;

    // Helper methods
    void configureDriver();
    void setupPins();
    bool checkAndReinitializeDriver();
    void handlePowerLoss();
    void checkStall();

    TMC5160Stepper& driver;   // Reference to the TMC5160 driver
    const uint8_t   csPin;    // Chip select pin
    const uint8_t   stepPin;  // Step pin
    const uint8_t   dirPin;   // Direction pin
    const uint8_t   enPin;    // Enable pin

    // Motor state variables
    bool          moving;
    bool          direction;
    unsigned long lastStepTime;
    int           stepCounter;
    uint16_t      runCurrent;
    uint16_t      holdCurrent;
    uint16_t      speed;
    uint16_t      acceleration;
    unsigned long lastTempPrintTime;
    int           lastTemperature;
};

#endif  // MOTOR_STATE_H