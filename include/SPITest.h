#ifndef SPI_TEST_H
#define SPI_TEST_H

#include <Arduino.h>
#include <SPI.h>
#include "config.h"

class SPITest {
public:
    SPITest();
    ~SPITest();

    // Initialize SPI test module
    void begin();

    // Test SPI communication for a specific driver
    bool testDriver(uint8_t driverNumber);

    // Get detailed test results
    String getLastTestResult() const {
        return lastTestResult;
    }

private:
    // Helper functions
    uint8_t getDriverCSPin(uint8_t driverNumber);
    void    printPinStates(uint8_t csPin);
    void    printTestResults(uint8_t driverNumber, uint8_t status, uint32_t readValue);

    // Store last test result
    String lastTestResult;
};

#endif  // SPI_TEST_H