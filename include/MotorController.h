#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <TMCStepper.h>
#include "SPIManager.h"
#include "config.h"

/**
 * @brief Motor Controller class for managing TMC5160 stepper motor driver
 *
 * This class implements a singleton pattern to manage the TMC5160 stepper motor driver.
 * It provides methods for motor control, current management, speed control, and status monitoring.
 */
class MotorController {
public:
    // Singleton instance access
    static MotorController& getInstance();

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

private:
    // Private constructor for singleton pattern
    MotorController();

    // Delete copy constructor and assignment operator
    MotorController(const MotorController&)            = delete;
    MotorController& operator=(const MotorController&) = delete;

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