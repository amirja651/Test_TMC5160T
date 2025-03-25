#include "SPIManager.h"

// Returns the singleton instance of SPIManager
SPIManager& SPIManager::getInstance() {
    static SPIManager instance;
    return instance;
}

// Initializes SPI communication with configured pins
void SPIManager::begin() {
    SPI.begin(Config::SPI::SCK, Config::SPI::MISO, Config::SPI::MOSI);
    pinMode(Config::SPI::CS, OUTPUT);
    deselectDevice();
}

// Performs a basic SPI communication test by sending a test pattern
void SPIManager::testCommunication() {
    selectDevice();
    transfer(Config::SPI::TEST_VALUE);
    deselectDevice();
}

// Activates the SPI device by setting CS pin low
void SPIManager::selectDevice() {
    digitalWrite(Config::SPI::CS, LOW);
}

// Deactivates the SPI device by setting CS pin high
void SPIManager::deselectDevice() {
    digitalWrite(Config::SPI::CS, HIGH);
}

// Performs a single byte SPI transfer
uint8_t SPIManager::transfer(uint8_t data) {
    return SPI.transfer(data);
}