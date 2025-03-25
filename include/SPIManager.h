#ifndef SPI_MANAGER_H
#define SPI_MANAGER_H

#include <Arduino.h>
#include <SPI.h>
#include "config.h"

class SPIManager {
public:
    static SPIManager& getInstance();
    void               begin();
    void               testCommunication();
    void               selectDevice();
    void               deselectDevice();
    uint8_t            transfer(uint8_t data);

private:
    SPIManager()                             = default;
    SPIManager(const SPIManager&)            = delete;
    SPIManager& operator=(const SPIManager&) = delete;
};

#endif