#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <TMCStepper.h>

#include "Helper/Pins.h"
#include "Helper/Types.h"

namespace MotionSystem
{
    enum class MotorType
    {
        NEMA11_HS13_1004H,  // 11HS13-1004H Stepper Motor
        P28SHD4611_12SK,    // P28SHD4611-12SK Hybrid Stepper Motor
        UNKNOWN
    };

    class TmcController
    {
    public:
        TmcController(const char* name, uint8_t csPin, uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                      MotorType motorType = MotorType::UNKNOWN, uint8_t mosiPin = Pins::SPI::MOSI,
                      uint8_t misoPin = Pins::SPI::MISO, uint8_t sckPin = Pins::SPI::SCK);
        void                begin();                          // Initialize the motor controller
        void                moveForward();                    // Move motor in forward direction
        void                moveReverse();                    // Move motor in reverse direction
        void                stop();                           // Stop motor movement
        void                update();                         // Update motor state and check status
        uint32_t            getDriverStatus();                // Get current driver status
        void                increaseRunCurrent();             // Increase motor running current
        void                decreaseRunCurrent();             // Decrease motor running current
        void                increaseHoldCurrent();            // Increase motor holding current
        void                decreaseHoldCurrent();            // Decrease motor holding current
        uint16_t            getRunCurrent() const;            // Get current running current value
        uint16_t            getHoldCurrent() const;           // Get current holding current value
        void                increaseSpeed();                  // Increase motor speed
        void                decreaseSpeed();                  // Decrease motor speed
        void                increaseAcceleration();           // Increase motor acceleration
        void                decreaseAcceleration();           // Decrease motor acceleration
        uint16_t            getSpeed() const;                 // Get current speed value
        uint16_t            getAcceleration() const;          // Get current acceleration value
        void                printDriverStatus();              // Print current driver status
        void                printDriverConfig();              // Print current driver configuration
        int                 getTemperature();                 // Get current driver temperature
        void                printTemperature();               // Print current temperature
        void                toggleStealthChop();              // Toggle stealth chop mode
        void                setStealthChopMode(bool enable);  // Set stealth chop mode
        bool                diagnoseTMC5160();
        bool                testCommunication(bool enableMessage = true);
        uint8_t             transfer(uint8_t data);
        void                enableDriver(bool enable);
        void                enableSPI();
        void                disableSPI();
        void                resetDriverState();
        void                setSpreadCycle(bool enable);         // Enable/disable SpreadCycle mode
        void                setRampMode(uint8_t mode);           // Set ramp mode (0: Positioning, 1: Velocity)
        void                setMaxSpeed(uint32_t speed);         // Set maximum speed
        void                setMaxAcceleration(uint32_t accel);  // Set maximum acceleration
        void                setMaxDeceleration(uint32_t decel);  // Set maximum deceleration
        void IRAM_ATTR      step();                              // Execute single step
        void                setDirection(bool forward);          // Set movement direction and update state
        uint32_t            calculateStepInterval(Types::Speed speed);
        Types::StepPosition micronsToSteps(Types::MicronPosition microns);
        Types::StepPosition pixelsToSteps(Types::PixelPosition pixels);
        String              getInstanceName();

    private:
        void           configureDriver();                       // Configure driver parameters
        void           setupPins();                             // Setup GPIO pins
        bool           checkAndReinitializeDriver();            // Check and reinitialize driver if needed
        void           printStatusRegister(uint32_t status);    // Print driver status register
        void           printErrorFlags(uint32_t status);        // Print error flags
        void           printStallGuardStatus(uint32_t status);  // Print stall guard status
        void           printDriverState(uint32_t status);       // Print driver state
        TMC5160Stepper driver;
        const char*    instanceName;
        uint8_t        csPin;
        uint8_t        stepPin;
        uint8_t        dirPin;
        uint8_t        enPin;
        uint8_t        mosiPin;
        uint8_t        misoPin;
        uint8_t        sckPin;
        MotorType      motorType;
        uint32_t       stepDelay;
        unsigned long  lastStepTime;            // Timestamp of last step
        int            stepCounter;             // Step counter for status updates
        uint16_t       runCurrent;              // Current running current value
        uint16_t       holdCurrent;             // Current holding current value
        uint16_t       speed;                   // Current speed in steps per second
        uint32_t       maxSpeed;                // Maximum speed
        uint16_t       acceleration;            // Current acceleration in steps per second squared
        uint32_t       maxAcceleration;         // Maximum acceleration
        uint32_t       maxDeceleration;         // Maximum deceleration
        unsigned long  lastTempPrintTime;       // Last temperature print timestamp
        int            lastTemperature;         // Last recorded temperature
        uint32_t       coolStepThreshold;       // CoolStep threshold value
        int8_t         stallGuardThreshold;     // StallGuard threshold value
        uint8_t        currentHoldDelay;        // Current hold delay
        uint8_t        rampMode;                // Current ramp mode
        bool           isMoving;                // Current movement state
        bool           direction;               // Current movement direction
        bool           diagnosticsEnabled;      // Whether advanced diagnostics are enabled
        bool           stallGuardFilter;        // StallGuard filter state
        bool           spreadCycleEnabled;      // SpreadCycle mode state
        bool           microstepInterpolation;  // Microstep interpolation state
    };
}  // namespace MotionSystem

#endif

/**
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
 */