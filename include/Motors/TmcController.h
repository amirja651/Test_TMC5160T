#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <TMCStepper.h>
#include "Config.h"

namespace MotionSystem
{
    class TmcController
    {
    public:
        TmcController(const char* name, uint8_t csPin, uint8_t stepPin, uint8_t dirPin, uint8_t enPin,
                      uint8_t mosiPin = Config::SPI::MOSI, uint8_t misoPin = Config::SPI::MISO,
                      uint8_t sckPin = Config::SPI::SCK);
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

    private:
        void           configureDriver2();                      // Configure driver parameters
        void           configureDriver();                       // Configure driver parameters
        void           setupPins();                             // Setup GPIO pins
        bool           checkAndReinitializeDriver();            // Check and reinitialize driver if needed
        void           printStatusRegister(uint32_t status);    // Print driver status register
        void           printErrorFlags(uint32_t status);        // Print error flags
        void           printStallGuardStatus(uint32_t status);  // Print stall guard status
        void           printDriverState(uint32_t status);       // Print driver state
        TMC5160Stepper driver;                                  // TMC5160 driver instance
        const char*    instanceName;                            // Name of this motor controller instance
        const uint8_t  csPin;                                   // Chip select pin
        const uint8_t  stepPin;                                 // Step pin
        const uint8_t  dirPin;                                  // Direction pin
        const uint8_t  enPin;                                   // Enable pin
        const uint8_t  mosiPin;                                 // MOSI pin
        const uint8_t  misoPin;                                 // MISO pin
        const uint8_t  sckPin;                                  // SCK pin
        const int      stepDelay;                               // Delay between steps
        unsigned long  lastStepTime;                            // Timestamp of last step
        int            stepCounter;                             // Step counter for status updates
        uint16_t       runCurrent;                              // Current running current value
        uint16_t       holdCurrent;                             // Current holding current value
        uint16_t       speed;                                   // Current speed in steps per second
        uint32_t       maxSpeed;                                // Maximum speed
        uint16_t       acceleration;                            // Current acceleration in steps per second squared
        uint32_t       maxAcceleration;                         // Maximum acceleration
        uint32_t       maxDeceleration;                         // Maximum deceleration
        unsigned long  lastTempPrintTime;                       // Last temperature print timestamp
        int            lastTemperature;                         // Last recorded temperature
        uint32_t       coolStepThreshold;                       // CoolStep threshold value
        int8_t         stallGuardThreshold;                     // StallGuard threshold value
        uint8_t        currentHoldDelay;                        // Current hold delay
        uint8_t        rampMode;                                // Current ramp mode
        bool           isMoving;                                // Current movement state
        bool           direction;                               // Current movement direction
        bool           diagnosticsEnabled;                      // Whether advanced diagnostics are enabled
        bool           stallGuardFilter;                        // StallGuard filter state
        bool           spreadCycleEnabled;                      // SpreadCycle mode state
        bool           microstepInterpolation;                  // Microstep interpolation state
    };
}  // namespace MotionSystem

#endif