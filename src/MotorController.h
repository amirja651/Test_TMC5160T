#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <TMCStepper.h>
#include "SPIManager.h"
#include "config.h"

class MotorController {
public:
    static MotorController& getInstance();
    void                    begin();
    void                    moveForward();
    void                    moveReverse();
    void                    stop();
    void                    update();
    uint32_t                getDriverStatus();

    // Current control methods
    void     increaseRunCurrent();
    void     decreaseRunCurrent();
    void     increaseHoldCurrent();
    void     decreaseHoldCurrent();
    uint16_t getRunCurrent() const;
    uint16_t getHoldCurrent() const;

    // Speed and acceleration control
    void     increaseSpeed();
    void     decreaseSpeed();
    void     increaseAcceleration();
    void     decreaseAcceleration();
    uint16_t getSpeed() const;
    uint16_t getAcceleration() const;

    // Status and configuration methods
    void printDriverStatus();
    void printDriverConfig();

    // Temperature monitoring
    int  getTemperature();
    void printTemperature();

private:
    MotorController();
    MotorController(const MotorController&)            = delete;
    MotorController& operator=(const MotorController&) = delete;

    void configureDriver();
    void setupPins();
    void step();
    bool checkAndReinitializeDriver();
    void handlePowerLoss();

    TMC5160Stepper            driver;
    bool                      isMoving;
    bool                      direction;
    const int                 stepDelay;
    unsigned long             lastStepTime;
    int                       stepCounter;
    static constexpr int      STATUS_PRINT_INTERVAL = 1000;
    static constexpr uint32_t INVALID_STATUS        = 0xFFFFFFFF;

    // Current settings with motor specifications constraints
    uint16_t                  runCurrent;
    uint16_t                  holdCurrent;
    static constexpr uint16_t CURRENT_STEP     = 100;   // mA
    static constexpr uint16_t MIN_CURRENT      = 100;   // mA
    static constexpr uint16_t MAX_RUN_CURRENT  = 1000;  // mA (1A max run current)
    static constexpr uint16_t MAX_HOLD_CURRENT = 500;   // mA (0.5A max hold current)

    // Speed and acceleration settings
    uint16_t                  speed;               // Steps per second
    uint16_t                  acceleration;        // Steps per second squared
    static constexpr uint16_t SPEED_STEP = 100;    // Steps/sec
    static constexpr uint16_t ACCEL_STEP = 100;    // Steps/sec²
    static constexpr uint16_t MIN_SPEED  = 100;    // Steps/sec
    static constexpr uint16_t MAX_SPEED  = 10000;  // Steps/sec
    static constexpr uint16_t MIN_ACCEL  = 100;    // Steps/sec²
    static constexpr uint16_t MAX_ACCEL  = 10000;  // Steps/sec²

    // Helper methods for status printing
    void printStatusRegister(uint32_t status);
    void printErrorFlags(uint32_t status);
    void printStallGuardStatus(uint32_t status);
    void printDriverState(uint32_t status);

    static constexpr int TEMP_PRINT_INTERVAL = 1000;  // Print temperature every 1 second
    unsigned long        lastTempPrintTime;
};

#endif