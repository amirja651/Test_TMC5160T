#ifndef SPI_MANAGER_H
#define SPI_MANAGER_H

#include <Arduino.h>
#include <SPI.h>
#include "config.h"

/**
 * @brief Singleton class for managing SPI communication with the TMC5160T stepper driver
 *
 * This class provides a centralized interface for SPI communication, ensuring
 * proper initialization and management of the SPI bus. It implements the singleton
 * pattern to prevent multiple instances from controlling the SPI bus simultaneously.
 */
class SPIManager {
public:
    /**
     * @brief Returns the singleton instance of SPIManager
     * @return Reference to the SPIManager instance
     */
    static SPIManager& getInstance();

    /**
     * @brief Initializes SPI communication with configured pins
     * Sets up SPI bus and configures CS pin as output
     */
    void begin();

    /**
     * @brief Performs a basic SPI communication test
     * Sends a test pattern (0x55) to verify SPI communication
     */
    void testCommunication();

    /**
     * @brief Activates the SPI device by setting CS pin low
     */
    void selectDevice();

    /**
     * @brief Deactivates the SPI device by setting CS pin high
     */
    void deselectDevice();

    /**
     * @brief Performs a single byte SPI transfer
     * @param data The byte to send over SPI
     * @return The byte received from the SPI device
     */
    uint8_t transfer(uint8_t data);

private:
    SPIManager()                             = default;
    SPIManager(const SPIManager&)            = delete;
    SPIManager& operator=(const SPIManager&) = delete;
};

#endif