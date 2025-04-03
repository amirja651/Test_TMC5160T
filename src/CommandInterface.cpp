#include "CommandInterface.h"
#include <SPI.h>
#include "Terminal.h"  // Add Terminal support for ESP32

// Color definitions using Terminal library
#define PIN_ON_COLOR  TerminalColor::Green
#define PIN_OFF_COLOR TerminalColor::Red
#define HEADER_COLOR  TerminalColor::White

CommandInterface::CommandInterface() {
    commandQueue      = NULL;
    commandTaskHandle = NULL;
}

CommandInterface::~CommandInterface() {
    if (commandTaskHandle != NULL) {
        vTaskDelete(commandTaskHandle);
    }
    if (commandQueue != NULL) {
        vQueueDelete(commandQueue);
    }
}

void CommandInterface::begin() {
    // Create command queue
    commandQueue = xQueueCreate(COMMAND_QUEUE_SIZE, COMMAND_BUFFER_SIZE);

    // Initialize Serial if not already initialized
    if (!Serial) {
        Serial.begin(115200);
    }

    // Create command processing task
    BaseType_t result = xTaskCreate(commandTask,              // Task function
                                    "CommandTask",            // Task name
                                    COMMAND_TASK_STACK_SIZE,  // Stack size
                                    this,                     // Task parameters
                                    COMMAND_TASK_PRIORITY,    // Priority
                                    &commandTaskHandle        // Task handle
    );

    if (result != pdPASS) {
        Serial.println("Failed to create command task!");
        return;
    }

    // Display initial menu
    displayCommandMenu();
}

void CommandInterface::commandTask(void* parameter) {
    CommandInterface* interface = (CommandInterface*)parameter;
    char              receivedCommand[COMMAND_BUFFER_SIZE];

    while (true) {
        // Check if there's data available in Serial
        if (Serial.available()) {
            // Read until newline or buffer is full
            size_t bytesRead = 0;
            while (Serial.available() && bytesRead < COMMAND_BUFFER_SIZE - 1) {
                char c = Serial.read();
                if (c == '\n' || c == '\r') {
                    break;
                }
                receivedCommand[bytesRead++] = c;
            }
            receivedCommand[bytesRead] = '\0';

            // Process the command
            interface->processCommand(receivedCommand);
        }

        // Small delay to prevent task starvation
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void CommandInterface::processCommand(const char* command) {
    // Convert command to lowercase for case-insensitive comparison
    String cmd = String(command);
    cmd.toLowerCase();

    if (cmd == "status" || cmd == "1") {
        displayPinStatus();
    } else if (cmd == "menu" || cmd == "2" || cmd == "help") {
        displayCommandMenu();
    } else if (cmd == "test1" || cmd == String(Config::CommandHandler::CMD_TEST_SPI_DRIVER1)) {
        Serial.println("\nTesting SPI communication for Driver 1...");
        if (testSPICommunication(1)) {
            Serial.println("Driver 1 SPI test: SUCCESS");
        } else {
            Serial.println("Driver 1 SPI test: FAILED");
        }
    } else if (cmd == "test2" || cmd == String(Config::CommandHandler::CMD_TEST_SPI_DRIVER2)) {
        Serial.println("\nTesting SPI communication for Driver 2...");
        if (testSPICommunication(2)) {
            Serial.println("Driver 2 SPI test: SUCCESS");
        } else {
            Serial.println("Driver 2 SPI test: FAILED");
        }
    } else if (cmd == "test3" || cmd == String(Config::CommandHandler::CMD_TEST_SPI_DRIVER3)) {
        Serial.println("\nTesting SPI communication for Driver 3...");
        if (testSPICommunication(3)) {
            Serial.println("Driver 3 SPI test: SUCCESS");
        } else {
            Serial.println("Driver 3 SPI test: FAILED");
        }
    } else if (cmd == "test4" || cmd == String(Config::CommandHandler::CMD_TEST_SPI_DRIVER4)) {
        Serial.println("\nTesting SPI communication for Driver 4...");
        if (testSPICommunication(4)) {
            Serial.println("Driver 4 SPI test: SUCCESS");
        } else {
            Serial.println("Driver 4 SPI test: FAILED");
        }
    } else {
        Serial.println("Unknown command. Type 'menu' to see available commands.");
    }
}

bool CommandInterface::testSPICommunication(uint8_t driverNumber) {
    // Get the CS pin for the specified driver
    uint8_t csPin;
    switch (driverNumber) {
        case 1:
            csPin = Config::TMC5160T_Driver::D1_CS;
            break;
        case 2:
            csPin = Config::TMC5160T_Driver::D2_CS;
            break;
        case 3:
            csPin = Config::TMC5160T_Driver::D3_CS;
            break;
        case 4:
            csPin = Config::TMC5160T_Driver::D4_CS;
            break;
        default:
            Serial.println("Invalid driver number");
            return false;
    }

    // End any existing SPI transaction
    SPI.endTransaction();

    // Configure SPI pins explicitly
    pinMode(Config::SPI::MOSI, OUTPUT);
    pinMode(Config::SPI::MISO, INPUT);
    pinMode(Config::SPI::SCK, OUTPUT);
    pinMode(csPin, OUTPUT);

    // Set initial states
    digitalWrite(Config::SPI::SCK, LOW);  // Start with SCK low for Mode 3
    digitalWrite(csPin, HIGH);            // Deselect driver

    // Initialize SPI with lower frequency for initial testing
    SPI.begin(Config::SPI::SCK, Config::SPI::MISO, Config::SPI::MOSI);
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));

    // Test reading GCONF register (0x00) first
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

    // Print initial read results
    Serial.printf("\nDriver %d Initial GCONF Read:\n", driverNumber);
    Serial.printf("Status Byte: 0x%02X\n", status);
    Serial.printf("GCONF Value: 0x%08X\n", (unsigned int)readValue);

    // If we got a valid response, try writing to GCONF
    if (readValue != 0xFFFFFFFF && readValue != 0x00000000) {
        uint8_t  writeAddr  = 0x00;        // GCONF without read bit
        uint32_t writeValue = 0x00000004;  // Default GCONF value

        // Write to GCONF
        digitalWrite(csPin, LOW);
        delayMicroseconds(100);

        SPI.transfer(writeAddr);
        SPI.transfer((writeValue >> 24) & 0xFF);
        SPI.transfer((writeValue >> 16) & 0xFF);
        SPI.transfer((writeValue >> 8) & 0xFF);
        SPI.transfer(writeValue & 0xFF);

        digitalWrite(csPin, HIGH);
        delayMicroseconds(100);

        // Read back GCONF to verify
        digitalWrite(csPin, LOW);
        delayMicroseconds(100);

        status    = SPI.transfer(readAddr);
        readValue = 0;
        for (int i = 0; i < 4; i++) {
            readValue = (readValue << 8) | SPI.transfer(0x00);
        }

        digitalWrite(csPin, HIGH);
        delayMicroseconds(100);

        // Print verification results
        Serial.printf("\nGCONF Write Verification:\n");
        Serial.printf("Status Byte: 0x%02X\n", status);
        Serial.printf("Value Written: 0x%08X\n", (unsigned int)writeValue);
        Serial.printf("Value Read Back: 0x%08X\n", (unsigned int)readValue);
    }

    // Check SPI pin states
    Serial.println("\nSPI Pin States:");
    Serial.printf("MOSI (GPIO%d): %s\n", Config::SPI::MOSI, digitalRead(Config::SPI::MOSI) ? "HIGH" : "LOW");
    Serial.printf("MISO (GPIO%d): %s\n", Config::SPI::MISO, digitalRead(Config::SPI::MISO) ? "HIGH" : "LOW");
    Serial.printf("SCK  (GPIO%d): %s\n", Config::SPI::SCK, digitalRead(Config::SPI::SCK) ? "HIGH" : "LOW");
    Serial.printf("CS   (GPIO%d): %s\n", csPin, digitalRead(csPin) ? "HIGH" : "LOW");

    SPI.endTransaction();

    // Check if we got a valid response
    bool isValid = (readValue != 0xFFFFFFFF && readValue != 0x00000000);

    if (!isValid) {
        Serial.println("\nWarning: SPI test failed");
        Serial.println("Possible issues:");
        if (readValue == 0xFFFFFFFF) {
            Serial.println("- MISO line stuck high");
            Serial.println("- No power to driver");
            Serial.println("- CS pin not properly connected");
        } else if (readValue == 0x00000000) {
            Serial.println("- MISO line stuck low");
            Serial.println("- Driver not responding");
        }
        Serial.println("- Check all power connections");
        Serial.println("- Verify SPI connections");
        Serial.println("- Check CS pin wiring");
    }

    return isValid;
}

void CommandInterface::displayPinStatus() {
    Serial.println("\n=== Pin Status ===");

    // Right side pins (GPIO 13-27, 32-39)
    Serial.println("\nRight Side Pins:");
    Serial.println("┌───────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┐");
    Serial.println("│  Pin  │ 23 │ 22 │ 21 │ 19 │ 18 │  5 │ 17 │ 16 │  4 │  2 │ 15 │");
    Serial.println("├───────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┤");
    Serial.print("│ State │");
    for (int pin : {23, 22, 21, 19, 18, 5, 17, 16, 4, 2, 15}) {
        Serial.printf(digitalRead(pin) ? " ON │" : "OFF │");
    }
    Serial.println("\n└───────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┘");

    // Left side pins (GPIO 0-12)
    Serial.println("\nLeft Side Pins:");
    Serial.println("┌───────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┬────┐");
    Serial.println("│  Pin  │ 36 │ 39 │ 34 │ 35 │ 32 │ 33 │ 25 │ 26 │ 27 │ 14 │ 12 │ 13 │");
    Serial.println("├───────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┼────┤");
    Serial.print("│ State │");
    for (int pin : {36, 39, 34, 35, 32, 33, 25, 26, 27, 14, 12, 13}) {
        Serial.printf(digitalRead(pin) ? " ON │" : "OFF │");
    }
    Serial.println("\n└───────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┴────┘");

    Serial.println("\nNote: TX/RX pins are reserved for serial communication");
    Serial.println("==========================================\n");
}

void CommandInterface::displayCommandMenu() {
    Serial.println("\n=== Command Menu ===");
    Serial.println("1. status  - Display current pin status");
    Serial.println("2. menu    - Display this command menu");
    Serial.println("3. test1   - Test SPI communication for Driver 1");
    Serial.println("4. test2   - Test SPI communication for Driver 2");
    Serial.println("5. test3   - Test SPI communication for Driver 3");
    Serial.println("6. test4   - Test SPI communication for Driver 4");
    Serial.println("===============\n");
}