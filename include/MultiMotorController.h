#ifndef MULTI_MOTOR_CONTROLLER_H
#define MULTI_MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <vector>
#include "MotorState.h"
#include "config.h"

/**
 * @brief Multi Motor Controller class for managing multiple TMC5160 stepper motors
 *
 * This class manages multiple TMC5160 stepper motors, where each motor has:
 * - 3 common pins (MOSI, MISO, SCK) shared across all motors
 * - 4 unique pins (CS, STEP, DIR, EN) per motor
 */
class MultiMotorController {
public:
    // Singleton instance access
    static MultiMotorController& getInstance();

    /**
     * @brief Constructor for MultiMotorController
     * @param numMotors Number of motors to manage
     * @param commonPins Array of 3 common pins [MOSI, MISO, SCK]
     * @param motorPins Array of motor-specific pins, where each motor has 4 pins [CS, STEP, DIR, EN]
     */
    MultiMotorController(uint8_t numMotors, const uint8_t* commonPins, const uint8_t* motorPins);

    /**
     * @brief Initialize all motors
     */
    void begin();

    /**
     * @brief Update state of all motors
     */
    void update();

    /**
     * @brief Move a specific motor forward
     * @param motorIndex Index of the motor to control
     */
    void moveForward(uint8_t motorIndex);

    /**
     * @brief Move a specific motor in reverse
     * @param motorIndex Index of the motor to control
     */
    void moveReverse(uint8_t motorIndex);

    /**
     * @brief Stop a specific motor
     * @param motorIndex Index of the motor to control
     */
    void stop(uint8_t motorIndex);

    /**
     * @brief Get driver status for a specific motor
     * @param motorIndex Index of the motor to check
     * @return Driver status register value
     */
    uint32_t getDriverStatus(uint8_t motorIndex) const;

    /**
     * @brief Increase run current for a specific motor
     * @param motorIndex Index of the motor to control
     */
    void increaseRunCurrent(uint8_t motorIndex);

    /**
     * @brief Decrease run current for a specific motor
     * @param motorIndex Index of the motor to control
     */
    void decreaseRunCurrent(uint8_t motorIndex);

    /**
     * @brief Increase hold current for a specific motor
     * @param motorIndex Index of the motor to control
     */
    void increaseHoldCurrent(uint8_t motorIndex);

    /**
     * @brief Decrease hold current for a specific motor
     * @param motorIndex Index of the motor to control
     */
    void decreaseHoldCurrent(uint8_t motorIndex);

    /**
     * @brief Get run current for a specific motor
     * @param motorIndex Index of the motor to check
     * @return Current run current value
     */
    uint16_t getRunCurrent(uint8_t motorIndex) const;

    /**
     * @brief Get hold current for a specific motor
     * @param motorIndex Index of the motor to check
     * @return Current hold current value
     */
    uint16_t getHoldCurrent(uint8_t motorIndex) const;

    /**
     * @brief Increase speed for a specific motor
     * @param motorIndex Index of the motor to control
     */
    void increaseSpeed(uint8_t motorIndex);

    /**
     * @brief Decrease speed for a specific motor
     * @param motorIndex Index of the motor to control
     */
    void decreaseSpeed(uint8_t motorIndex);

    /**
     * @brief Increase acceleration for a specific motor
     * @param motorIndex Index of the motor to control
     */
    void increaseAcceleration(uint8_t motorIndex);

    /**
     * @brief Decrease acceleration for a specific motor
     * @param motorIndex Index of the motor to control
     */
    void decreaseAcceleration(uint8_t motorIndex);

    /**
     * @brief Get speed for a specific motor
     * @param motorIndex Index of the motor to check
     * @return Current speed value
     */
    uint16_t getSpeed(uint8_t motorIndex) const;

    /**
     * @brief Get acceleration for a specific motor
     * @param motorIndex Index of the motor to check
     * @return Current acceleration value
     */
    uint16_t getAcceleration(uint8_t motorIndex) const;

    /**
     * @brief Print driver status for a specific motor
     * @param motorIndex Index of the motor to check
     */
    void printDriverStatus(uint8_t motorIndex) const;

    /**
     * @brief Print driver configuration for a specific motor
     * @param motorIndex Index of the motor to check
     */
    void printDriverConfig(uint8_t motorIndex) const;

    /**
     * @brief Get temperature for a specific motor
     * @param motorIndex Index of the motor to check
     * @return Current temperature in Celsius
     */
    int getTemperature(uint8_t motorIndex) const;

    /**
     * @brief Print temperature for a specific motor
     * @param motorIndex Index of the motor to check
     */
    void printTemperature(uint8_t motorIndex) const;

    /**
     * @brief Toggle stealth chop mode for a specific motor
     * @param motorIndex Index of the motor to control
     */
    void toggleStealthChop(uint8_t motorIndex);

    // Additional methods from MotorController
    void printErrorFlags(uint32_t status) const;
    void printStallGuardStatus(uint32_t status) const;
    void printDriverState(uint32_t status) const;

    // SPI Management Methods
    /**
     * @brief Test SPI communication with the motor drivers
     */
    void testCommunication();

    /**
     * @brief Select a specific motor's CS pin
     * @param motorIndex Index of the motor to select
     */
    void selectDevice(uint8_t motorIndex);

    /**
     * @brief Deselect a specific motor's CS pin
     * @param motorIndex Index of the motor to deselect
     */
    void deselectDevice(uint8_t motorIndex);

    /**
     * @brief Perform a single byte SPI transfer
     * @param data The byte to send over SPI
     * @return The byte received from the SPI device
     */
    uint8_t transfer(uint8_t data);

private:
    // Private constructor for singleton pattern
    MultiMotorController();

    // Delete copy constructor and assignment operator
    MultiMotorController(const MultiMotorController&)            = delete;
    MultiMotorController& operator=(const MultiMotorController&) = delete;

    /**
     * @brief Validate motor index
     * @param motorIndex Index to validate
     * @return true if index is valid
     */
    bool isValidMotorIndex(uint8_t motorIndex) const;

    /**
     * @brief Get motor state for a specific index
     * @param motorIndex Index of the motor
     * @return Reference to the motor state
     */
    MotorState& getMotorState(uint8_t motorIndex);

    /**
     * @brief Get motor state for a specific index (const version)
     * @param motorIndex Index of the motor
     * @return Const reference to the motor state
     */
    const MotorState& getMotorState(uint8_t motorIndex) const;

    const uint8_t  numMotors;   // Number of motors being managed
    const uint8_t* commonPins;  // Array of common pins [MOSI, MISO, SCK]
    const uint8_t* motorPins;   // Array of motor-specific pins [CS, STEP, DIR, EN] for each motor

    std::vector<TMC5160Stepper> drivers;  // Array of TMC5160 drivers
    std::vector<MotorState>     motors;   // Array of motor states
};

#endif