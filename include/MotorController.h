#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include "config.h"

/**
 * @brief Motor Controller class for managing TMC5160 stepper motor driver
 *
 * This class implements a controller for the TMC5160 stepper motor driver.
 * Multiple instances can be created, each controlling a separate motor.
 */
class MotorController {
public:
    /**
     * @brief Constructor for MotorController
     * @param csPin Chip select pin for this motor
     * @param stepPin Step pin for this motor
     * @param dirPin Direction pin for this motor
     * @param enPin Enable pin for this motor
     * @param mosiPin MOSI pin for SPI (optional, defaults to Config::SPI::MOSI)
     * @param misoPin MISO pin for SPI (optional, defaults to Config::SPI::MISO)
     * @param sckPin SCK pin for SPI (optional, defaults to Config::SPI::SCK)
     */
    MotorController(uint8_t csPin, uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                    uint8_t mosiPin = Config::SPI::MOSI, uint8_t misoPin = Config::SPI::MISO,
                    uint8_t sckPin = Config::SPI::SCK);

    // Core motor control methods
    void     begin();            // Initialize the motor controller
    void     moveForward();      // Move motor in forward direction
    void     moveReverse();      // Move motor in reverse direction
    void     stop();             // Stop motor movement
    void     update();           // Update motor state and check status
    uint32_t getDriverStatus();  // Get current driver status

    // Current control methods
    void     increaseRunCurrent();    // Increase motor running current
    void     decreaseRunCurrent();    // Decrease motor running current
    void     increaseHoldCurrent();   // Increase motor holding current
    void     decreaseHoldCurrent();   // Decrease motor holding current
    uint16_t getRunCurrent() const;   // Get current running current value
    uint16_t getHoldCurrent() const;  // Get current holding current value

    // Speed and acceleration control
    void     increaseSpeed();          // Increase motor speed
    void     decreaseSpeed();          // Decrease motor speed
    void     increaseAcceleration();   // Increase motor acceleration
    void     decreaseAcceleration();   // Decrease motor acceleration
    uint16_t getSpeed() const;         // Get current speed value
    uint16_t getAcceleration() const;  // Get current acceleration value

    // Status and configuration methods
    void printDriverStatus();  // Print current driver status
    void printDriverConfig();  // Print current driver configuration

    // Temperature monitoring
    int  getTemperature();    // Get current driver temperature
    void printTemperature();  // Print current temperature

    void toggleStealthChop();  // Toggle stealth chop mode

    /**
     * @brief Performs a basic SPI communication test
     * Sends a test pattern (0x55) to verify SPI communication
     * @return true if communication is successful, false otherwise
     */
    bool testCommunication();

    /**
     * @brief Performs a single byte SPI transfer
     * @param data The byte to send over SPI
     * @return The byte received from the SPI device
     */
    uint8_t transfer(uint8_t data);

    /**
     * @brief Enable the motor driver by setting EN pin low
     */
    void enableDriver();

    /**
     * @brief Disable the motor driver by setting EN pin high
     */
    void disableDriver();

    /**
     * @brief Enable SPI communication by setting CS pin low
     */
    void enableSPI();

    /**
     * @brief Disable SPI communication by setting CS pin high
     */
    void disableSPI();

    /**
     * @brief Set step pin high
     */
    void stepHigh();

    /**
     * @brief Set step pin low
     */
    void stepLow();

    /**
     * @brief Set direction pin high (forward)
     */
    void dirHigh();

    /**
     * @brief Set direction pin low (reverse)
     */
    void dirLow();

private:
    // Internal configuration methods
    void configureDriver();             // Configure driver parameters
    void setupPins();                   // Setup GPIO pins
    void step();                        // Execute single step
    bool checkAndReinitializeDriver();  // Check and reinitialize driver if needed
    void handlePowerLoss();             // Handle power loss situation
    void checkStall();                  // Check for motor stall condition

    // Driver instance and state variables
    TMC5160Stepper driver;        // TMC5160 driver instance
    bool           isMoving;      // Current movement state
    bool           direction;     // Current movement direction
    const int      stepDelay;     // Delay between steps
    unsigned long  lastStepTime;  // Timestamp of last step
    int            stepCounter;   // Step counter for status updates

    // Pin assignments
    const uint8_t csPin;    // Chip select pin
    const uint8_t stepPin;  // Step pin
    const uint8_t dirPin;   // Direction pin
    const uint8_t enPin;    // Enable pin
    const uint8_t mosiPin;  // MOSI pin
    const uint8_t misoPin;  // MISO pin
    const uint8_t sckPin;   // SCK pin

    // Current settings
    uint16_t runCurrent;   // Current running current value
    uint16_t holdCurrent;  // Current holding current value

    // Speed and acceleration settings
    uint16_t speed;         // Current speed in steps per second
    uint16_t acceleration;  // Current acceleration in steps per second squared

    // Status printing helper methods
    void printStatusRegister(uint32_t status);    // Print driver status register
    void printErrorFlags(uint32_t status);        // Print error flags
    void printStallGuardStatus(uint32_t status);  // Print stall guard status
    void printDriverState(uint32_t status);       // Print driver state

    // Temperature monitoring variables
    unsigned long lastTempPrintTime;  // Last temperature print timestamp
    int           lastTemperature;    // Last recorded temperature
};

#endif