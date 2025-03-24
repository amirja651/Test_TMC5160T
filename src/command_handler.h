#pragma once
#include <Arduino.h>
#include "motor_controller.h"

// Forward declarations
struct DriverStatus;
struct MotorStatus;
struct ThermalStatus;

class CommandHandler {
public:
    static constexpr size_t MAX_COMMAND_LENGTH = 32;

    // Command structure
    struct CommandMap {
        const char* en_cmd;
        void (CommandHandler::*handler)(const char*);
        const char* description;
    };

    explicit CommandHandler(MotorController& motor_controller)
        : motor(motor_controller), bufferIndex(0) {
        clearBuffer();
    }

    void processSerialInput();

    void executeDetailedStatus() {
        Serial.println("\n=== Detailed Status ===");
        displayDriverStatus();
        displayMotorStatus();
        displayThermalStatus();
        displayPowerStatus();
        displayErrorFlags();
        displayStepperConfig();
    }

private:
    MotorController& motor;
    char             commandBuffer[MAX_COMMAND_LENGTH];
    size_t           bufferIndex;

    // Command table
    static const CommandMap COMMANDS[];

    // Command handlers
    void executeMove(const char* params);
    void executeSetSpeed(const char* params);
    void executeSetCurrent(const char* params);
    void executeStatus(const char* params);
    void executeDetailedStatus(const char* params);
    void executeEnable(const char* params);
    void executeDisable(const char* params);
    void executeTest(const char* params);
    void executeHelp(const char* params);
    void executeStop(const char* params);
    void executeTemperature(const char* params);
    void executeAutoTune(const char* params);

    // Display methods
    void displayDriverStatus();
    void displayMotorStatus();
    void displayThermalStatus();
    void displayPowerStatus();
    void displayErrorFlags();
    void displayStepperConfig();

    // Helper methods
    void              handleCommand();
    void              clearBuffer();
    void              printError(const char* message);
    bool              parseFloat(const char* str, float& value);
    bool              parseInt(const char* str, int32_t& value);
    const CommandMap* findCommand(const char* cmd) const;
    const char*       getChopperModeStr(uint8_t mode);
    const char*       getRunModeStr(uint8_t mode);
    void              printProgressBar(float value, float max, uint8_t length = 20);
    void              printTemperatureStatus(float temp, float warning, float max);
};