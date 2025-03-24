#include "command_handler.h"

// Define command mapping table
const CommandHandler::CommandMap CommandHandler::COMMANDS[] = {
    {"m", &CommandHandler::executeMove, "Move motor (steps direction)"},
    {"s", &CommandHandler::executeSetSpeed, "Set speed (steps/sec)"},
    {"c", &CommandHandler::executeSetCurrent, "Set current (mA)"},
    {"t", &CommandHandler::executeStatus, "Show status"},
    {"d", &CommandHandler::executeDetailedStatus, "Show detailed status"},
    {"e", &CommandHandler::executeEnable, "Enable motor"},
    {"x", &CommandHandler::executeDisable, "Disable motor"},
    {"p", &CommandHandler::executeTest, "Run connection test"},
    {"h", &CommandHandler::executeHelp, "Show help"},
    {"q", &CommandHandler::executeStop, "Emergency stop"},
    {"temp", &CommandHandler::executeTemperature, "Read motor temperature"},
    {"tune", &CommandHandler::executeAutoTune, "Run auto-tuning process"},
    {nullptr, nullptr, nullptr}  // End marker
};

const CommandHandler::CommandMap* CommandHandler::findCommand(const char* cmd) const {
    if (!cmd)
        return nullptr;

    for (const auto& command : COMMANDS) {
        if (!command.en_cmd)
            break;  // End of commands
        if (strcasecmp(cmd, command.en_cmd) == 0) {
            return &command;
        }
    }
    return nullptr;
}

void CommandHandler::processSerialInput() {
    const uint32_t MAX_PROCESS_TIME = 1000;  // microseconds
    uint32_t       startTime        = micros();

    while (Serial.available() > 0) {
        // Check processing time limit
        if (micros() - startTime > MAX_PROCESS_TIME) {
            break;  // Prevent blocking too long
        }

        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (bufferIndex > 0) {
                commandBuffer[bufferIndex] = '\0';
                handleCommand();
                clearBuffer();
            }
        } else if (bufferIndex < MAX_COMMAND_LENGTH - 1) {
            commandBuffer[bufferIndex++] = c;
        }
    }
}

void CommandHandler::handleCommand() {
    char* command = strtok(commandBuffer, " ");
    char* params  = strtok(nullptr, "");

    if (!command)
        return;

    // Echo the command
    Serial.print("\nReceived command: ");
    Serial.print(command);
    if (params) {
        Serial.print(" ");
        Serial.print(params);
    }
    Serial.println();

    // Convert to lowercase for case-insensitive comparison
    for (char* p = command; *p; ++p) {
        *p = tolower(*p);
    }

    const CommandMap* cmd = findCommand(command);
    if (cmd) {
        (this->*(cmd->handler))(params);
    } else {
        printError("Unknown command. Type 'h' for help");
    }
}

/**
 * @brief Processes and executes the move command
 * @param params String containing steps and direction parameters
 */
void CommandHandler::executeMove(const char* params) {
    // Add thermal check before movement
    if (motor.getThermalStatus().currentTemp >= motor.getThermalStatus().warningTemp) {
        printError("Movement restricted - High temperature");
        return;
    }

    if (!params) {
        printError("Missing parameters. Usage: m <steps> <r/l>");
        return;
    }

    // Parse steps
    int32_t steps;
    if (!parseInt(params, steps)) {
        printError("Invalid steps parameter");
        return;
    }

    // Parse direction
    char* dirStr = strchr(params, ' ');
    if (dirStr) {
        dirStr++;  // Skip the space
    }

    bool clockwise = false;

    if (dirStr) {
        // Convert to lowercase for case-insensitive comparison
        for (char* p = dirStr; *p; ++p) {
            *p = tolower(*p);
        }

        // Check for 'r' or 'l' command
        if (strcmp(dirStr, "r") == 0) {
            clockwise = true;
        } else if (strcmp(dirStr, "l") == 0) {
            clockwise = false;
        } else {
            printError("Invalid direction. Use 'r' for right or 'l' for left");
            return;
        }

        // Debug message
        Serial.print("Direction: ");
        Serial.println(clockwise ? "right" : "left");
    } else {
        printError("Missing direction parameter. Use 'r' for right or 'l' for left");
        return;
    }

    if (!motor.isReady()) {
        printError("Motor not ready");
        return;
    }

    // Enable motor before movement
    if (!motor.isEnabled()) {
        if (!motor.enable()) {
            printError("Failed to enable motor");
            return;
        }
        Serial.println("Motor enabled");
    }

    // Set direction
    if (!motor.setDirection(clockwise)) {
        printError("Failed to set direction");
        return;
    }
    Serial.println("Direction set");

    // Calculate target position
    int32_t currentPos = motor.getMotorStatus().currentPosition;
    int32_t targetPos  = clockwise ? currentPos + steps : currentPos - steps;

    // Debug message
    Serial.print("Current position: ");
    Serial.println(currentPos);
    Serial.print("Target position: ");
    Serial.println(targetPos);

    // Move to target position
    Serial.print("Moving ");
    Serial.print(steps);
    Serial.println(clockwise ? " steps right" : " steps left");

    if (!motor.moveToPosition(targetPos)) {
        printError("Failed to start movement");
        return;
    }

    // Wait for movement to complete with timeout
    const uint32_t startTime = millis();
    const uint32_t timeout   = 5000;  // 5 second timeout

    while (motor.getMotorStatus().currentPosition != targetPos) {
        if (millis() - startTime > timeout) {
            printError("Movement timeout");
            motor.emergencyStop();
            return;
        }
        delay(10);
    }

    // Movement completed successfully
    Serial.println("Movement completed");
}

/**
 * @brief Sets the motor speed
 * @param params String containing speed value in steps/sec
 */
void CommandHandler::executeSetSpeed(const char* params) {
    if (!params) {
        printError("Missing parameter. Usage: speed <value>");
        return;
    }

    int32_t speed;
    if (!parseInt(params, speed)) {
        printError("Invalid speed parameter");
        return;
    }

    Serial.print("Setting speed to: ");
    Serial.println(speed);
    motor.setSpeed(speed);
}

void CommandHandler::executeStatus(const char* params) {
    (void)params;  // Unused parameter

    Serial.println("\nMotor Status:");
    Serial.print("Enabled: ");
    Serial.println(motor.isEnabled() ? "Yes" : "No");

    auto motorStatus = motor.getMotorStatus();
    Serial.print("Position: ");
    Serial.println(motorStatus.currentPosition);
    Serial.print("Speed: ");
    Serial.println(motorStatus.currentSpeed);
}

void CommandHandler::executeHelp(const char* params) {
    (void)params;  // Unused parameter

    Serial.println("\nCommands (single letter):");
    Serial.println("m <steps> <r/l> - Move");
    Serial.println("s <speed>       - Speed");
    Serial.println("c <current>     - Current");
    Serial.println("t               - Status");
    Serial.println("d               - Detailed Status");
    Serial.println("e               - Enable/Disable");
    Serial.println("x               - Disable");
    Serial.println("p               - Test Connection");
    Serial.println("h               - Show Help");
    Serial.println("q               - Emergency Stop");
    Serial.println("temp            - Read motor temperature");
    Serial.println("tune            - Run auto-tuning process");
}

void CommandHandler::executeTest(const char* params) {
    (void)params;  // Unused parameter

    auto result = motor.testConnection();
    if (result.success) {
        Serial.println("Connection test passed");
        Serial.print("Driver version: 0x");
        Serial.println(result.version, HEX);
    } else {
        Serial.println("Connection test failed:");
        Serial.println(result.errorMessage);
    }
}

void CommandHandler::executeEnable(const char* params) {
    (void)params;  // Unused parameter

    if (motor.enable()) {
        Serial.println("Motor enabled");
    } else {
        printError("Failed to enable motor");
    }
}

void CommandHandler::executeDetailedStatus(const char* params) {
    (void)params;  // Unused parameter
    Serial.println("\n=== Detailed Status ===");
    displayDriverStatus();
    displayMotorStatus();
    displayThermalStatus();
    displayPowerStatus();
    displayErrorFlags();
    displayStepperConfig();
}

void CommandHandler::displayDriverStatus() {
    Serial.println("\n--- Driver Status ---");

    // Get driver status
    auto drvStatus = motor.getDriverStatus();

    Serial.print("Driver State: ");
    if (motor.isEnabled()) {
        Serial.println("ENABLED");
    } else {
        Serial.println("DISABLED");
    }

    Serial.print("Chopper Mode: ");
    Serial.println(getChopperModeStr(drvStatus.chopperMode));

    Serial.print("Microstep Resolution: 1/");
    Serial.println(motor.getMicrostepResolution());

    Serial.print("Current Setting: ");
    Serial.print(motor.getCurrentSetting());
    Serial.println(" mA");
}

void CommandHandler::displayMotorStatus() {
    Serial.println("\n--- Motor Status ---");

    auto motorStatus = motor.getMotorStatus();

    Serial.print("Current Position: ");
    Serial.print(motorStatus.currentPosition);
    Serial.println(" steps");

    Serial.print("Current Speed: ");
    Serial.print(motorStatus.currentSpeed);
    Serial.println(" steps/sec");

    Serial.print("Target Position: ");
    Serial.print(motorStatus.targetPosition);
    Serial.println(" steps");

    Serial.print("Running Mode: ");
    Serial.println(getRunModeStr(motorStatus.runMode));

    // Display load percentage with progress bar
    Serial.print("Motor Load: ");
    float loadPercent = motorStatus.motorLoad;
    printProgressBar(loadPercent, 100.0f);
    Serial.print(" ");
    Serial.print(loadPercent, 1);
    Serial.println("%");
}

void CommandHandler::displayThermalStatus() {
    auto status = motor.getThermalStatus();
    Serial.println("\nThermal Status:");
    Serial.print("Temperature: ");
    Serial.print(status.currentTemp);
    Serial.println("°C");
    Serial.print("Warning Level: ");
    Serial.print(status.protectionLevel);
    Serial.println("%");
    if (status.protectionActive) {
        Serial.println("WARNING: Temperature protection active!");
    }
}

void CommandHandler::displayPowerStatus() {
    Serial.println("\n--- Power Status ---");

    auto powerStatus = motor.getPowerStatus();

    Serial.print("Supply Voltage: ");
    Serial.print(powerStatus.supplyVoltage, 1);
    Serial.println("V");

    Serial.print("Motor Current: ");
    Serial.print(powerStatus.motorCurrent, 0);
    Serial.println(" mA");

    Serial.print("Power Efficiency: ");
    Serial.print(powerStatus.efficiency, 1);
    Serial.println("%");
}

void CommandHandler::displayErrorFlags() {
    Serial.println("\n--- Error Flags ---");

    auto errorFlags = motor.getErrorFlags();

    if (errorFlags.any()) {
        if (errorFlags.overtemperature)
            Serial.println("! OVERTEMPERATURE");
        if (errorFlags.shortCircuit)
            Serial.println("! SHORT CIRCUIT");
        if (errorFlags.openLoad)
            Serial.println("! OPEN LOAD");
        if (errorFlags.overCurrent)
            Serial.println("! OVERCURRENT");
        if (errorFlags.stallGuard)
            Serial.println("! STALL DETECTED");
    } else {
        Serial.println("No errors reported");
    }
}

void CommandHandler::displayStepperConfig() {
    Serial.println("\n--- Stepper Configuration ---");

    auto config = motor.getStepperConfig();

    Serial.print("Step Mode: ");
    Serial.print("1/");
    Serial.println(config.microsteps);

    Serial.print("Current Scale: ");
    Serial.print(config.currentScale);
    Serial.println("%");

    Serial.print("Hold Current: ");
    Serial.print(config.holdCurrent);
    Serial.println(" mA");

    Serial.print("Run Current: ");
    Serial.print(config.runCurrent);
    Serial.println(" mA");
}

const char* CommandHandler::getChopperModeStr(uint8_t mode) {
    switch (mode) {
        case 0:
            return "SpreadCycle";
        case 1:
            return "StealthChop";
        default:
            return "Unknown";
    }
}

const char* CommandHandler::getRunModeStr(uint8_t mode) {
    switch (mode) {
        case 0:
            return "Stopped";
        case 1:
            return "Positioning";
        case 2:
            return "Velocity";
        case 3:
            return "Hold";
        default:
            return "Unknown";
    }
}

void CommandHandler::printProgressBar(float value, float max, uint8_t length) {
    uint8_t filled = (value / max) * length;
    Serial.print("[");
    for (uint8_t i = 0; i < length; i++) {
        if (i < filled) {
            Serial.print("=");
        } else {
            Serial.print(" ");
        }
    }
    Serial.print("]");
}

void CommandHandler::printTemperatureStatus(float temp, float warning, float max) {
    if (temp >= max) {
        Serial.print("CRITICAL!");
    } else if (temp >= warning) {
        Serial.print("WARNING!");
    } else {
        Serial.print("Normal");
    }
}

void CommandHandler::clearBuffer() {
    memset(commandBuffer, 0, MAX_COMMAND_LENGTH);
    bufferIndex = 0;
}

void CommandHandler::printError(const char* message) {
    Serial.print("Error: ");
    Serial.println(message);
}

bool CommandHandler::parseFloat(const char* str, float& value) {
    char* endPtr;
    value = strtof(str, &endPtr);
    return endPtr != str;
}

bool CommandHandler::parseInt(const char* str, int32_t& value) {
    char* endPtr;
    value = strtol(str, &endPtr, 10);
    return endPtr != str;
}

void CommandHandler::executeSetCurrent(const char* params) {
    if (!params) {
        printError("Missing parameter. Usage: c <mA>");
        return;
    }

    int32_t current;
    if (!parseInt(params, current)) {
        printError("Invalid current value");
        return;
    }

    if (current < 0 || current > 2000) {  // Current limit
        printError("Current must be between 0 and 2000 mA");
        return;
    }

    if (motor.setMotorCurrent(current)) {
        Serial.print("Motor current set to: ");
        Serial.print(current);
        Serial.println(" mA");
    } else {
        printError("Failed to set motor current");
    }
}

void CommandHandler::executeStop(const char* params) {
    (void)params;  // Unused parameter

    if (!motor.isReady()) {
        printError("Motor not ready");
        return;
    }

    if (motor.emergencyStop()) {
        Serial.println("Motor stopped");
    } else {
        printError("Failed to stop motor");
    }
}

void CommandHandler::executeDisable(const char* params) {
    (void)params;  // Unused parameter

    if (motor.disable()) {
        Serial.println("Motor disabled");
    } else {
        printError("Failed to disable motor");
    }
}

void CommandHandler::executeTemperature(const char* command) {
    // Get temperature from motor controller
    float temperature     = motor.getTemperature();
    bool  overTempWarning = motor.overTemperaturePreWarning();
    bool  overTempFlag    = motor.overTemperatureFlag();

    // Print temperature information
    Serial.println("\nTemperature Status:");
    Serial.print("Current Temperature: ");
    Serial.print(temperature, 1);
    Serial.println("°C");

    if (overTempWarning) {
        Serial.println("WARNING: Temperature pre-warning active!");
    }
    if (overTempFlag) {
        Serial.println("ERROR: Over-temperature condition detected!");
    }

    // Print thermal protection status
    auto thermalStatus = motor.getThermalStatus();
    Serial.print("Protection Level: ");
    Serial.print(thermalStatus.protectionLevel);
    Serial.println("%");
    Serial.print("Warning Temperature: ");
    Serial.print(thermalStatus.warningTemp);
    Serial.println("°C");
    Serial.print("Maximum Temperature: ");
    Serial.print(thermalStatus.maxTemp);
    Serial.println("°C");
}

void CommandHandler::executeAutoTune(const char* params) {
    (void)params;  // Unused parameter

    Serial.println("\nStarting Auto-Tuning Process...");
    Serial.println("This process will take several minutes.");
    Serial.println("Please ensure the motor is properly mounted and can move freely.");
    Serial.println("Press any key to continue or 'q' to cancel...");

    // Wait for user confirmation
    while (!Serial.available()) {
        delay(100);
    }

    char c = Serial.read();
    if (c == 'q' || c == 'Q') {
        Serial.println("Auto-tuning cancelled by user.");
        return;
    }

    // Clear any remaining characters
    while (Serial.available()) {
        Serial.read();
    }

    // Run auto-tuning
    if (motor.autoTune()) {
        Serial.println("\nAuto-tuning completed successfully!");
        Serial.println("New settings have been applied.");
    } else {
        printError("Auto-tuning failed");
    }
}