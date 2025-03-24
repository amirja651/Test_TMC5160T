#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include "config.h"

// SPI Pins for ESP32
#define SPI_MOSI   Config::SPI::MOSI
#define SPI_MISO   Config::SPI::MISO
#define SPI_SCK    Config::SPI::SCK
#define TMC_CS_PIN Config::SPI::CS

// Motor control pins
#define STEP_PIN Config::TMC5160T_Driver::STEP_PIN
#define DIR_PIN  Config::TMC5160T_Driver::DIR_PIN
#define EN_PIN   Config::TMC5160T_Driver::EN_PIN

// Motor driver config
#define TMC_CURRENT_MA Config::TMC5160T_Driver::TMC_CURRENT_MA
#define MICROSTEPS     Config::TMC5160T_Driver::MICROSTEPS

TMC5160Stepper driver = TMC5160Stepper(TMC_CS_PIN, SPI_MOSI, SPI_MISO, SPI_SCK);

// Global variables for motion control
bool          isMoving     = false;
bool          direction    = true;  // true = forward, false = reverse
const int     stepDelay    = 500;   // microseconds
unsigned long lastStepTime = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);  // Startup delay for stability

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    // SPI Test
    pinMode(TMC_CS_PIN, OUTPUT);
    digitalWrite(TMC_CS_PIN, HIGH);

    // Test SPI communication
    uint8_t test_value = 0x55;
    digitalWrite(TMC_CS_PIN, LOW);
    SPI.transfer(test_value);
    digitalWrite(TMC_CS_PIN, HIGH);

    Serial.println("SPI Test completed");

    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);

    digitalWrite(EN_PIN, HIGH);  // Disable motor initially
    delay(200);

    driver.begin();  // Initialize driver communication
    delay(200);

    // Basic driver configuration
    driver.GCONF(0x0);  // Basic configuration
    delay(100);

    // Set motor current to appropriate value
    driver.rms_current(1000);  // Set to 1000mA
    delay(100);

    driver.microsteps(16);  // Set microstepping to 16
    delay(100);

    // Driver timing settings
    driver.toff(5);  // Off time setting
    driver.blank_time(24);
    driver.iholddelay(6);
    driver.TPOWERDOWN(10);
    delay(100);

    // Check driver communication
    uint32_t gconf = driver.GCONF();
    Serial.print("GCONF: 0x");
    Serial.println(gconf, HEX);

    uint32_t drv_status = driver.DRV_STATUS();
    Serial.print("DRV_STATUS: 0x");
    Serial.println(drv_status, HEX);

    delay(1000);
    digitalWrite(EN_PIN, LOW);  // Enable motor
    delay(200);
}

void loop() {
    // Read serial input
    if (Serial.available() > 0) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'f':  // Move forward
                direction = true;
                isMoving  = true;
                digitalWrite(DIR_PIN, HIGH);
                Serial.println("Moving Forward");
                break;
            case 'r':  // Move reverse
                direction = false;
                isMoving  = true;
                digitalWrite(DIR_PIN, LOW);
                Serial.println("Moving Reverse");
                break;
            case 's':  // Stop movement
                isMoving = false;
                Serial.println("Stopped");
                break;
        }
    }

    // Motor motion control
    if (isMoving) {
        if (micros() - lastStepTime >= stepDelay) {
            digitalWrite(STEP_PIN, HIGH);
            delayMicroseconds(10);
            digitalWrite(STEP_PIN, LOW);
            lastStepTime = micros();

            // Print status every 1000 steps
            static int stepCounter = 0;
            if (++stepCounter >= 1000) {
                uint32_t status = driver.DRV_STATUS();
                Serial.print("DRV_STATUS: 0x");
                Serial.println(status, HEX);
                stepCounter = 0;
            }
        }
    }
}