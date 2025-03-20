#include <Arduino.h>
#include <SPI.h>

// SPI Pin configuration for ESP32
#define SPI_MOSI 23  // Pin MOSI
#define SPI_MISO 19  // Pin MISO
#define SPI_SCK  18  // Pin SCK
#define CS_PIN   5   // Chip Select pin

// STEP/DIR pins
#define STEP_PIN 13  // STEP pin
#define DIR_PIN  12  // DIR pin
#define EN_PIN   14  // Enable pin (active LOW)

// Register addresses
#define REG_GCONF      0x00  // Global configuration
#define REG_GSTAT      0x01  // Global status
#define REG_IHOLD_IRUN 0x10  // Motor current control
#define REG_CHOPCONF   0x6C  // Chopper configuration
#define REG_DRVSTATUS  0x6F  // Driver status

// Position control registers
#define REG_RAMPMODE 0x20  // 0=Position, 1=Velocity, 2=Hold
#define REG_XACTUAL  0x21  // Actual position
#define REG_VACTUAL  0x22  // Actual velocity
#define REG_VSTART   0x23  // Start velocity
#define REG_A1       0x24  // First acceleration
#define REG_V1       0x25  // First velocity
#define REG_AMAX     0x26  // Maximum acceleration
#define REG_VMAX     0x27  // Maximum velocity
#define REG_DMAX     0x28  // Maximum deceleration
#define REG_D1       0x2A  // First deceleration
#define REG_VSTOP    0x2B  // Stop velocity
#define REG_XTARGET  0x2D  // Target position

// Additional TMC5160T registers
#define REG_COOLCONF   0x6D  // Coolstep configuration
#define REG_PWMCONF    0x70  // PWM configuration
#define REG_ENCMODE    0x38  // Encoder mode configuration
#define REG_TPOWERDOWN 0x11  // Power down delay
#define REG_TPWMTHRS   0x13  // Upper velocity for StealthChop
#define REG_TCOOLTHRS  0x14  // Lower velocity for CoolStep
#define REG_THIGH      0x15  // Upper velocity for StealthChop
#define REG_SGTHRS     0x40  // StallGuard threshold
#define REG_SGCONF     0x41  // StallGuard configuration

// Motor-specific configuration for 11HS13-1004H
#define MOTOR_STEPS_PER_REV 200    // 1.8 degrees per step
#define MOTOR_RATED_CURRENT 1000   // 1.0A rated current
#define MOTOR_PHASE_RES     2.1f   // 2.1 ohms phase resistance
#define MOTOR_INDUCTANCE    1.5f   // 1.5mH inductance
#define MOTOR_MAX_SPEED     3000   // Maximum speed in steps/second
#define MOTOR_MAX_ACCEL     50000  // Maximum acceleration in steps/second^2

// Current settings (TMC5160 uses 31 current steps, 1A = ~3.2 units)
#define MOTOR_CURRENT_RUN  (MOTOR_RATED_CURRENT / 31)  // Running current
#define MOTOR_CURRENT_HOLD (MOTOR_CURRENT_RUN * 0.5)   // 50% holding current

// Read from a TMC5160 register
uint32_t readRegister(uint8_t reg) {
    uint32_t data = 0;

    digitalWrite(CS_PIN, LOW);
    SPI.transfer(reg & 0x7F);  // Read operation (MSB=0)

    // Read 4 bytes
    data = ((uint32_t)SPI.transfer(0x00) << 24);
    data |= ((uint32_t)SPI.transfer(0x00) << 16);
    data |= ((uint32_t)SPI.transfer(0x00) << 8);
    data |= ((uint32_t)SPI.transfer(0x00));

    digitalWrite(CS_PIN, HIGH);

    return data;
}

// Write to a TMC5160 register
void writeRegister(uint8_t reg, uint32_t value) {
    digitalWrite(CS_PIN, LOW);

    SPI.transfer(reg | 0x80);  // Write operation (MSB=1)

    // Write 4 bytes (MSB first)
    SPI.transfer((value >> 24) & 0xFF);
    SPI.transfer((value >> 16) & 0xFF);
    SPI.transfer((value >> 8) & 0xFF);
    SPI.transfer(value & 0xFF);

    digitalWrite(CS_PIN, HIGH);
    delayMicroseconds(10);  // Small delay
}

// Function to configure motor current
void setMotorCurrent(uint8_t ihold, uint8_t irun) {
    // IHOLD_IRUN register: IHOLD[4:0], IRUN[12:8], IHOLDDELAY[19:16]
    uint32_t ihold_irun = 0;
    ihold_irun |= (ihold & 0x1F);        // IHOLD (bits 0-4)
    ihold_irun |= ((irun & 0x1F) << 8);  // IRUN (bits 8-12)
    ihold_irun |= (2 << 16);             // IHOLDDELAY (bits 16-19)

    writeRegister(REG_IHOLD_IRUN, ihold_irun);

    Serial.print("Motor current set to - Running: ");
    Serial.print(irun * 0.31, 2);  // Convert to amps (approximate)
    Serial.print("A, Holding: ");
    Serial.print(ihold * 0.31, 2);
    Serial.println("A");
}

// Function to configure chopper settings
void configureChopper() {
    // CHOPCONF register configuration
    uint32_t chopconf = 0;
    chopconf |= 3 << 0;   // TOFF = 3: Chopper off time
    chopconf |= 4 << 4;   // HSTRT = 4: Hysteresis start
    chopconf |= 0 << 7;   // HEND = 0: Hysteresis end
    chopconf |= 0 << 12;  // TBL = 0: Blank time
    chopconf |= 1 << 14;  // VSENSE = 1: High sensitivity
    chopconf |= 0 << 15;  // MRES = 0: Native 256 microsteps
    chopconf |= 1 << 24;  // DISS2VS = 1: Short to VS protection
    chopconf |= 1 << 27;  // DISS2G = 1: Short to GND protection
    writeRegister(REG_CHOPCONF, chopconf);
}

// Function to configure CoolStep
void configureCoolStep() {
    // COOLCONF register configuration
    uint32_t coolconf = 0;
    coolconf |= 1 << 0;   // SEMIN = 1: Minimum CoolStep current
    coolconf |= 0 << 5;   // SEUP = 0: Current increment steps
    coolconf |= 0 << 8;   // SEMAX = 0: Maximum CoolStep current
    coolconf |= 0 << 13;  // SEDN = 0: Current decrement steps
    coolconf |= 0 << 16;  // SEIMIN = 0: Minimum current for smart current control
    writeRegister(REG_COOLCONF, coolconf);
}

// Function to configure StallGuard
void configureStallGuard() {
    // SGCONF register configuration
    uint32_t sgconf = 0;
    sgconf |= 5 << 0;   // CS = 5: Current scale
    sgconf |= 0 << 8;   // SGT = 0: StallGuard threshold
    sgconf |= 0 << 16;  // SFILT = 0: StallGuard filter
    writeRegister(REG_SGCONF, sgconf);

    // Set StallGuard threshold
    writeRegister(REG_SGTHRS, 10);  // Adjust based on application
}

// Function to configure power settings
void configurePowerSettings() {
    // PWMCONF register configuration
    uint32_t pwmconf = 0;
    pwmconf |= 1 << 0;   // PWM_OFS = 1: PWM offset
    pwmconf |= 1 << 8;   // PWM_GRAD = 1: PWM gradient
    pwmconf |= 0 << 16;  // PWM_FREQ = 0: PWM frequency
    pwmconf |= 0 << 18;  // PWM_AUTOSCALE = 0: Disable autoscaling
    pwmconf |= 0 << 19;  // PWM_AUTOGRAD = 0: Disable autogradient
    pwmconf |= 0 << 24;  // FREEWHEEL = 0: Normal operation
    writeRegister(REG_PWMCONF, pwmconf);

    // Set power down delay
    writeRegister(REG_TPOWERDOWN, 10);  // 10 * 2^18 clocks
}

// Function to run full diagnostics
void runDiagnostics() {
    Serial.println("\n=== Running Full Diagnostics ===");

    // 1. Check driver status
    uint32_t drvstatus = readRegister(REG_DRVSTATUS);
    Serial.print("Driver Status: 0x");
    Serial.println(drvstatus, HEX);

    // Check for errors
    if ((drvstatus & 0xF0000000) != 0) {
        Serial.println("ERROR: Driver reports issues!");
        if (drvstatus & (1UL << 31))
            Serial.println("- Motor open load B");
        if (drvstatus & (1UL << 30))
            Serial.println("- Motor open load A");
        if (drvstatus & (1UL << 29))
            Serial.println("- Short to GND B");
        if (drvstatus & (1UL << 28))
            Serial.println("- Short to GND A");
        if (drvstatus & (1UL << 27))
            Serial.println("- Short VB");
        if (drvstatus & (1UL << 26))
            Serial.println("- Short VA");
        if (drvstatus & (1UL << 25))
            Serial.println("- Overtemperature warning");
        if (drvstatus & (1UL << 24))
            Serial.println("- Overtemperature shutdown");
    } else {
        Serial.println("Driver status OK");
    }

    // 2. Check global status
    uint32_t gstat = readRegister(REG_GSTAT);
    Serial.print("Global Status: 0x");
    Serial.println(gstat, HEX);
    if (gstat & 0x07) {
        if (gstat & 0x01)
            Serial.println("- Reset occurred");
        if (gstat & 0x02)
            Serial.println("- Driver error occurred");
        if (gstat & 0x04)
            Serial.println("- SG2 active");
    }

    // 3. Verify current settings
    uint32_t ihold_irun = readRegister(REG_IHOLD_IRUN);
    float    irun       = ((ihold_irun >> 8) & 0x1F) * 0.31;
    float    ihold      = (ihold_irun & 0x1F) * 0.31;
    Serial.print("Motor currents - Run: ");
    Serial.print(irun, 2);
    Serial.print("A, Hold: ");
    Serial.print(ihold, 2);
    Serial.println("A");

    // 4. Check chopper settings
    uint32_t chopconf = readRegister(REG_CHOPCONF);
    Serial.print("Chopper config: 0x");
    Serial.println(chopconf, HEX);

    // 5. Check velocity
    int32_t velocity = readRegister(REG_VACTUAL);
    Serial.print("Current velocity: ");
    Serial.println(velocity);

    Serial.println("=== Diagnostics Complete ===\n");
}

// Function to reset and reconfigure driver
void resetAndReconfigure() {
    Serial.println("\n=== Resetting and Reconfiguring Driver ===");

    // 1. Disable the driver
    digitalWrite(EN_PIN, HIGH);
    delay(100);

    // 2. Clear all error flags
    writeRegister(REG_GSTAT, 0x07);

    // 3. Configure basic settings
    uint32_t gconf = 0x00000004;  // Enable I_scale_analog
    writeRegister(REG_GCONF, gconf);

    // 4. Configure motor current
    setMotorCurrent(MOTOR_CURRENT_HOLD, MOTOR_CURRENT_RUN);

    // 5. Configure chopper
    configureChopper();

    // 6. Configure CoolStep
    configureCoolStep();

    // 7. Configure StallGuard
    configureStallGuard();

    // 8. Configure power settings
    configurePowerSettings();

    // 9. Configure motion parameters
    writeRegister(REG_RAMPMODE, 0);   // Position mode
    writeRegister(REG_VSTART, 1);     // Start velocity
    writeRegister(REG_A1, 1000);      // First acceleration
    writeRegister(REG_V1, 50000);     // First velocity
    writeRegister(REG_AMAX, 5000);    // Maximum acceleration
    writeRegister(REG_VMAX, 200000);  // Maximum velocity
    writeRegister(REG_DMAX, 5000);    // Maximum deceleration
    writeRegister(REG_D1, 1000);      // First deceleration
    writeRegister(REG_VSTOP, 10);     // Stop velocity

    // 10. Re-enable the driver
    digitalWrite(EN_PIN, LOW);
    delay(100);

    Serial.println("Driver reset and reconfigured successfully");
    Serial.println("Running diagnostics to verify configuration...");
    runDiagnostics();
}

// Setup function
void setup() {
    // Initialize serial communication
    Serial.begin(115200);
    while (!Serial) {
        ;  // Wait for serial port to connect
    }

    Serial.println("\n=======================================");
    Serial.println("TMC5160T Pro Driver Configuration");
    Serial.println("Motor: 11HS13-1004H (1.8deg, 1.0A)");
    Serial.println("=======================================");

    // Configure pins
    pinMode(CS_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);

    // Initial pin states
    digitalWrite(CS_PIN, HIGH);  // Deactivate CS
    digitalWrite(STEP_PIN, LOW);
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(EN_PIN, HIGH);  // Driver disabled initially

    // Initialize SPI with TMC5160 settings
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

    delay(100);  // Allow time for initialization

    // Perform full driver configuration
    resetAndReconfigure();

    // Print initial menu
    Serial.println("\nDriver initialized and ready.");
    Serial.println("Available commands:");
    Serial.println("  'd' - Run full diagnostics");
    Serial.println("  's' - Show driver status");
    Serial.println("  'c' - Move motor CW one revolution (STEP/DIR)");
    Serial.println("  'a' - Move motor CCW one revolution (STEP/DIR)");
    Serial.println("  'z' - Move motor CW using SPI control");
    Serial.println("  'x' - Move motor CCW using SPI control");
    Serial.println("  '+' - Increase motor current by 10%");
    Serial.println("  '-' - Decrease motor current by 10%");
    Serial.println("  'e' - Toggle enable/disable motor driver");
    Serial.println("  'r' - Reset and reconfigure driver");
    Serial.println("  'h' - Show this menu");
    Serial.println("=======================================");
}

// Function to move motor using direct SPI control
void moveMotorDirectSPI(int32_t steps, bool clockwise) {
    Serial.println("\nMoving motor using direct SPI control...");

    // 1. Initial driver configuration
    writeRegister(REG_GCONF, 0x00000004);  // Basic configuration (I_scale_analog=1)
    delay(10);

    // 2. Clear any error flags
    writeRegister(REG_GSTAT, 0x07);
    delay(10);

    // 3. Read current status and position
    uint32_t drvstatus = readRegister(REG_DRVSTATUS);
    Serial.print("Driver status: 0x");
    Serial.println(drvstatus, HEX);

    // 4. Reset actual position to 0
    writeRegister(REG_XACTUAL, 0);
    delay(10);

    // 5. Calculate target position
    int32_t targetPos = clockwise ? steps : -steps;
    Serial.print("Target position: ");
    Serial.println(targetPos);

    // 6. Configure motion parameters with conservative values
    writeRegister(REG_RAMPMODE, 0);  // Position mode
    delay(5);
    writeRegister(REG_VSTART, 1);  // Start velocity - very slow
    delay(5);
    writeRegister(REG_A1, 50);  // First acceleration - very gentle
    delay(5);
    writeRegister(REG_V1, 500);  // First velocity - slow
    delay(5);
    writeRegister(REG_AMAX, 100);  // Max acceleration - gentle
    delay(5);
    writeRegister(REG_VMAX, 1000);  // Max velocity - moderate
    delay(5);
    writeRegister(REG_DMAX, 100);  // Max deceleration - gentle
    delay(5);
    writeRegister(REG_D1, 50);  // First deceleration - very gentle
    delay(5);
    writeRegister(REG_VSTOP, 10);  // Stop velocity - slow
    delay(5);

    // 7. Verify some key registers were properly set
    uint32_t vmax     = readRegister(REG_VMAX);
    uint32_t rampmode = readRegister(REG_RAMPMODE);
    Serial.print("VMAX: ");
    Serial.print(vmax);
    Serial.print(", RAMPMODE: ");
    Serial.println(rampmode);

    // 8. Set target position to initiate movement
    Serial.println("Initiating movement...");
    writeRegister(REG_XTARGET, targetPos);
    delay(10);

    // 9. Monitor the movement
    bool          moving     = true;
    unsigned long startTime  = millis();
    int           checkCount = 0;

    while (moving && (millis() - startTime < 20000)) {  // 20-second timeout
        checkCount++;
        int32_t actualPos = (int32_t)readRegister(REG_XACTUAL);
        int32_t velocity  = (int32_t)readRegister(REG_VACTUAL);

        Serial.print("Position: ");
        Serial.print(actualPos);
        Serial.print("/");
        Serial.print(targetPos);
        Serial.print(", Velocity: ");
        Serial.println(velocity);

        if (checkCount >= 10) {
            // Every 10 checks, read driver status to see if there are any errors
            uint32_t status = readRegister(REG_DRVSTATUS);
            Serial.print("DRVSTATUS: 0x");
            Serial.println(status, HEX);
            checkCount = 0;
        }

        // Check if position reached or timeout
        if (abs(actualPos - targetPos) < 5) {  // Allow small position error
            Serial.println("Target position reached or very close!");
            moving = false;
        } else if (velocity == 0 && actualPos != targetPos) {
            // Motor stopped but didn't reach target - possible stall or obstacle
            Serial.println("Motor stopped but didn't reach target!");
            moving = false;
        }

        delay(500);  // Check every 500ms
    }

    if (moving) {
        Serial.println("Movement timed out. Motor may still be moving or stalled.");
    }

    // Final position
    int32_t finalPos = (int32_t)readRegister(REG_XACTUAL);
    Serial.print("Final position: ");
    Serial.println(finalPos);
}

// Function to move the motor one revolution
void moveMotorOneRevolution(bool clockwise) {
    Serial.println(clockwise ? "\nMoving motor one revolution clockwise..."
                             : "\nMoving motor one revolution counter-clockwise...");

    // Enable the driver
    digitalWrite(EN_PIN, LOW);
    delay(10);

    // Set direction
    digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);

    // For a 1.8° motor, 200 steps = 1 revolution
    const int STEPS_PER_REV = 200;

    // Move the motor
    for (int i = 0; i < STEPS_PER_REV; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(1000);  // 1ms high time
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(1000);  // 1ms low time

        // Print progress every 20 steps
        if (i % 20 == 0) {
            Serial.print("Steps completed: ");
            Serial.println(i);
        }
    }

    Serial.println("Movement complete");

    // Keep driver enabled
}

// Function to display all important registers
void readAllRegisters() {
    Serial.println("\n--- Register Status ---");

    uint32_t gconf      = readRegister(REG_GCONF);
    uint32_t gstat      = readRegister(REG_GSTAT);
    uint32_t ihold_irun = readRegister(REG_IHOLD_IRUN);
    uint32_t chopconf   = readRegister(REG_CHOPCONF);
    uint32_t drvstatus  = readRegister(REG_DRVSTATUS);

    Serial.print("GCONF: 0x");
    Serial.println(gconf, HEX);

    Serial.print("GSTAT: 0x");
    Serial.println(gstat, HEX);

    Serial.print("IHOLD_IRUN: 0x");
    Serial.println(ihold_irun, HEX);
    Serial.print("  IHOLD: ");
    Serial.print((ihold_irun & 0x1F) * 0.31, 2);
    Serial.print("A, IRUN: ");
    Serial.print(((ihold_irun >> 8) & 0x1F) * 0.31, 2);
    Serial.println("A");

    Serial.print("CHOPCONF: 0x");
    Serial.println(chopconf, HEX);

    Serial.print("DRVSTATUS: 0x");
    Serial.println(drvstatus, HEX);
    // Check for errors
    if ((drvstatus & 0xF0000000) != 0) {
        Serial.println("  WARNING: Driver status shows errors!");
        if (drvstatus & (1UL << 31))
            Serial.println("  - Motor open load B");
        if (drvstatus & (1UL << 30))
            Serial.println("  - Motor open load A");
        if (drvstatus & (1UL << 29))
            Serial.println("  - Short to GND B");
        if (drvstatus & (1UL << 28))
            Serial.println("  - Short to GND A");
        if (drvstatus & (1UL << 27))
            Serial.println("  - Short VB");
        if (drvstatus & (1UL << 26))
            Serial.println("  - Short VA");
        if (drvstatus & (1UL << 25))
            Serial.println("  - Overtemperature warning");
        if (drvstatus & (1UL << 24))
            Serial.println("  - Overtemperature shutdown");
    } else {
        Serial.println("  No errors detected");
    }
}

// Main loop
void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();

        switch (command) {
            case '+':  // Increase current by ~10%
            {
                uint8_t new_irun  = min(31, current_irun + 1);  // Maximum is 31
                uint8_t new_ihold = min(31, current_ihold + 1);
                Serial.print("Increasing current to - Running: ");
                Serial.print(new_irun * 0.31, 2);
                Serial.print("A, Holding: ");
                Serial.print(new_ihold * 0.31, 2);
                Serial.println("A");
                setMotorCurrent(new_ihold, new_irun);
            } break;

            case '-':  // Decrease current by ~10%
            {
                uint8_t new_irun  = max(1, current_irun - 1);  // Minimum is 1
                uint8_t new_ihold = max(1, current_ihold - 1);
                Serial.print("Decreasing current to - Running: ");
                Serial.print(new_irun * 0.31, 2);
                Serial.print("A, Holding: ");
                Serial.print(new_ihold * 0.31, 2);
                Serial.println("A");
                setMotorCurrent(new_ihold, new_irun);
            } break;

            case 's':  // Show driver status
            case 'S':
                readAllRegisters();
                break;

            case 'c':  // Move clockwise
            case 'C':
                moveMotorOneRevolution(true);
                break;

            case 'a':  // Move counter-clockwise
            case 'A':
                moveMotorOneRevolution(false);
                break;

            case 'z':  // Move using SPI (clockwise)
            case 'Z':
                moveMotorDirectSPI(200, true);
                break;

            case 'x':  // Move using SPI (counter-clockwise)
            case 'X':
                moveMotorDirectSPI(200, false);
                break;

            case 'd':  // Run full diagnostics
            case 'D':
                runDiagnostics();
                break;

            case 'r':  // Reset and reconfigure
            case 'R':
                resetAndReconfigure();
                break;

            case 'e':  // Enable/disable driver
            case 'E': {
                static bool driverEnabled = true;
                driverEnabled             = !driverEnabled;
                digitalWrite(EN_PIN, driverEnabled ? LOW : HIGH);
                Serial.println(driverEnabled ? "Driver ENABLED" : "Driver DISABLED");
            } break;

            case 'h':  // Help menu
            case 'H':
                Serial.println("\nCommands:");
                Serial.println("  'd' - Run full diagnostics");
                Serial.println("  's' - Show driver status");
                Serial.println("  'c' - Move motor CW one revolution (STEP/DIR)");
                Serial.println("  'a' - Move motor CCW one revolution (STEP/DIR)");
                Serial.println("  'z' - Move motor CW using SPI control");
                Serial.println("  'x' - Move motor CCW using SPI control");
                Serial.println("  '+' - Increase motor current by 10%");
                Serial.println("  '-' - Decrease motor current by 10%");
                Serial.println("  'e' - Toggle enable/disable motor driver");
                Serial.println("  'r' - Reset and reconfigure driver");
                Serial.println("  'h' - Show this menu");
                break;

            default:
                break;
        }

        // Clear any remaining characters in buffer
        while (Serial.available()) {
            Serial.read();
        }
    }

    delay(10);  // Small delay to prevent CPU hogging
}