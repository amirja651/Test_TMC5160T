#include "SPIManager.h"

SPIManager& SPIManager::getInstance() {
    static SPIManager instance;
    return instance;
}

void SPIManager::begin() {
    SPI.begin(Config::SPI::SCK, Config::SPI::MISO, Config::SPI::MOSI);
    pinMode(Config::SPI::CS, OUTPUT);
    deselectDevice();
}

void SPIManager::testCommunication() {
    uint8_t test_value = 0x55;
    selectDevice();
    transfer(test_value);
    deselectDevice();
}

void SPIManager::selectDevice() {
    digitalWrite(Config::SPI::CS, LOW);
}

void SPIManager::deselectDevice() {
    digitalWrite(Config::SPI::CS, HIGH);
}

uint8_t SPIManager::transfer(uint8_t data) {
    return SPI.transfer(data);
}