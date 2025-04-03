#include <Arduino.h>
#include "SPITest.h"

SPITest spiTest;

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        ;  // Wait for serial port to connect
    }

    Serial.println("\nTMC5160T SPI Test Program");
    Serial.println("=========================");

    // Initialize SPI test module
    spiTest.begin();

    // Test each driver
    for (int driver = 1; driver <= 4; driver++) {
        Serial.println("\nTesting Driver " + String(driver));
        Serial.println("----------------");

        bool result = spiTest.testDriver(driver);
        Serial.println(spiTest.getLastTestResult());

        if (result) {
            Serial.println("Driver " + String(driver) + " test PASSED");
        } else {
            Serial.println("Driver " + String(driver) + " test FAILED");
        }

        delay(1000);  // Wait between tests
    }
}

void loop() {
    // Nothing to do in loop
    delay(1000);
}