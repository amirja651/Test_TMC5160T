#include "SPITest.h"

SPITest::SPITest() {
    lastTestResult = "";
}

SPITest::~SPITest() {
    SPI.endTransaction();
}

void SPITest::begin() {
    // Configure SPI pins explicitly
    pinMode(Config::SPI::MOSI, OUTPUT);
    pinMode(Config::SPI::MISO, INPUT);
    pinMode(Config::SPI::SCK, OUTPUT);

    // Set initial states
    digitalWrite(Config::SPI::SCK, LOW);  // Start with SCK low for Mode 3

    // Initialize SPI with lower frequency for initial testing
    SPI.begin(Config::SPI::SCK, Config::SPI::MISO, Config::SPI::MOSI);
}

uint8_t SPITest::getDriverCSPin(uint8_t driverNumber) {
    switch (driverNumber) {
        case 1:
            return Config::TMC5160T_Driver::D1_CS;
        /*case 2:
            return Config::TMC5160T_Driver::D2_CS;
        case 3:
            return Config::TMC5160T_Driver::D3_CS;
        case 4:
            return Config::TMC5160T_Driver::D4_CS;*/
        default:
            return 0xFF;
    }
}

void SPITest::printPinStates(uint8_t csPin) {
    lastTestResult += "\nSPI Pin States:\n";
    lastTestResult +=
        "MOSI (GPIO" + String(Config::SPI::MOSI) + "): " + (digitalRead(Config::SPI::MOSI) ? "HIGH" : "LOW") + "\n";
    lastTestResult +=
        "MISO (GPIO" + String(Config::SPI::MISO) + "): " + (digitalRead(Config::SPI::MISO) ? "HIGH" : "LOW") + "\n";
    lastTestResult +=
        "SCK  (GPIO" + String(Config::SPI::SCK) + "): " + (digitalRead(Config::SPI::SCK) ? "HIGH" : "LOW") + "\n";
    lastTestResult += "CS   (GPIO" + String(csPin) + "): " + (digitalRead(csPin) ? "HIGH" : "LOW") + "\n";
}

void SPITest::printTestResults(uint8_t driverNumber, uint8_t status, uint32_t readValue) {
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "\nDriver %d Test Results:\n", driverNumber);
    lastTestResult += buffer;
    snprintf(buffer, sizeof(buffer), "Status Byte: 0x%02X\n", status);
    lastTestResult += buffer;
    snprintf(buffer, sizeof(buffer), "Register Value: 0x%08X\n", (unsigned int)readValue);
    lastTestResult += buffer;
}

bool SPITest::testDriver(uint8_t driverNumber) {
    lastTestResult = "";  // Clear previous results

    uint8_t csPin = getDriverCSPin(driverNumber);
    if (csPin == 0xFF) {
        lastTestResult = "Invalid driver number";
        return false;
    }

    // Configure CS pin
    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);  // Deselect driver

    // Begin SPI transaction
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));

    // Test reading GCONF register (0x00)
    uint8_t  readAddr  = 0x00 | 0x80;  // GCONF with read bit
    uint32_t readValue = 0;

    // Read GCONF
    digitalWrite(csPin, LOW);
    delayMicroseconds(100);

    uint8_t status = SPI.transfer(readAddr);
    for (int i = 0; i < 4; i++) {
        readValue = (readValue << 8) | SPI.transfer(0x00);
    }

    digitalWrite(csPin, HIGH);
    delayMicroseconds(100);

    // Print test results
    printTestResults(driverNumber, status, readValue);
    printPinStates(csPin);

    // End SPI transaction
    SPI.endTransaction();

    // Check if we got a valid response
    bool isValid = (readValue != 0xFFFFFFFF && readValue != 0x00000000);

    if (!isValid) {
        lastTestResult += "\nWarning: SPI test failed\n";
        lastTestResult += "Possible issues:\n";
        if (readValue == 0xFFFFFFFF) {
            lastTestResult += "- MISO line stuck high\n";
            lastTestResult += "- No power to driver\n";
            lastTestResult += "- CS pin not properly connected\n";
        } else if (readValue == 0x00000000) {
            lastTestResult += "- MISO line stuck low\n";
            lastTestResult += "- Driver not responding\n";
        }
        lastTestResult += "- Check all power connections\n";
        lastTestResult += "- Verify SPI connections\n";
        lastTestResult += "- Check CS pin wiring\n";
    }

    return isValid;
}