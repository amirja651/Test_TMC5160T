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

// Motor-specific configuration for stepper motor
#define MOTOR_STEPS_PER_REV 200    // 1.8 degrees per step
#define MOTOR_RATED_CURRENT 500    // 0.5A rated current
#define MOTOR_PHASE_RES     3.5f   // 3.5 ohms phase resistance
#define MOTOR_INDUCTANCE    0.90f  // 0.90mH inductance
#define MOTOR_MAX_SPEED     3000   // Maximum speed in steps/second
#define MOTOR_MAX_ACCEL     50000  // Maximum acceleration in steps/second^2

// Current settings (TMC5160 uses 31 current steps, 1A = ~3.2 units)
// For 0.5A rated current: 0.5A / 0.31A ≈ 1.6 units
#define MOTOR_CURRENT_RUN  2  // ~0.62A running current (slightly above rated for margin)
#define MOTOR_CURRENT_HOLD 1  // ~0.31A holding current (about 50% of run current)

// Function prototypes
uint32_t readRegister(uint8_t reg);
void     writeRegister(uint8_t reg, uint32_t value);
void     setMotorCurrent(uint8_t ihold, uint8_t irun);
void     configureChopper();
void     configureCoolStep();
void     configureStallGuard();
void     configurePowerSettings();
void     runDiagnostics();
void     resetAndReconfigure();
void     moveMotorDirectSPI(int32_t steps, bool clockwise);
void     moveMotorOneRevolution(bool clockwise);
void     readAllRegisters();
void     monitorSPITransaction(uint8_t reg);
bool     verifySPICommunication();

// Global variables for current settings
static uint32_t g_current_settings = 0;
static uint8_t  g_current_irun     = 0;
static uint8_t  g_current_ihold    = 0;

// Function to adjust motor current by a percentage
void adjustMotorCurrent(bool increase) {
    // Read current settings
    g_current_settings = readRegister(REG_IHOLD_IRUN);
    g_current_irun     = (g_current_settings >> 8) & 0x1F;
    g_current_ihold    = g_current_settings & 0x1F;
    uint8_t new_irun, new_ihold;

    if (increase) {
        // Increase current by ~10%
        new_irun  = (g_current_irun >= 31) ? 31 : g_current_irun + 1;
        new_ihold = (g_current_ihold >= 31) ? 31 : g_current_ihold + 1;
        Serial.print("Increasing current to - Running: ");
    } else {
        // Decrease current by ~10%
        new_irun  = (g_current_irun <= 1) ? 1 : g_current_irun - 1;
        new_ihold = (g_current_ihold <= 1) ? 1 : g_current_ihold - 1;
        Serial.print("Decreasing current to - Running: ");
    }

    Serial.print(new_irun * 0.31, 2);
    Serial.print("A, Holding: ");
    Serial.print(new_ihold * 0.31, 2);
    Serial.println("A");

    setMotorCurrent(new_ihold, new_irun);
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
    chopconf |= 4 << 0;   // TOFF = 4: Chopper off time (increased for better torque)
    chopconf |= 5 << 4;   // HSTRT = 5: Hysteresis start (increased)
    chopconf |= 2 << 7;   // HEND = 2: Hysteresis end (increased for better torque)
    chopconf |= 2 << 12;  // TBL = 2: Blank time (increased)
    chopconf |= 1 << 14;  // VSENSE = 1: High sensitivity
    chopconf |= 4 << 15;  // MRES = 4: 16 microsteps (changed from 256)
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
    gconf |= 0x00000001;          // Enable stealthChop
    writeRegister(REG_GCONF, gconf);

    // 4. Configure motor current - reduced for smoother operation
    setMotorCurrent(3, 6);  // ~0.93A run, ~0.46A hold

    // 5. Configure chopper for smooth operation
    uint32_t chopconf = 0;
    chopconf |= 4 << 0;   // TOFF = 4: Chopper off time
    chopconf |= 3 << 4;   // HSTRT = 3: Moderate hysteresis start
    chopconf |= 2 << 7;   // HEND = 2: Moderate hysteresis end
    chopconf |= 1 << 12;  // TBL = 1: Moderate blank time
    chopconf |= 1 << 14;  // VSENSE = 1: High sensitivity for better current control
    chopconf |= 3 << 15;  // MRES = 3: 32 microsteps (good balance of smoothness and torque)
    chopconf |= 1 << 24;  // DISS2VS = 1: Short to VS protection
    chopconf |= 1 << 27;  // DISS2G = 1: Short to GND protection
    writeRegister(REG_CHOPCONF, chopconf);

    // 6. Configure power stage for smooth operation
    uint32_t pwmconf = 0;
    pwmconf |= 8 << 0;   // PWM_OFS = 8: Increased offset for stable current
    pwmconf |= 4 << 8;   // PWM_GRAD = 4: Smoother gradient
    pwmconf |= 1 << 16;  // PWM_FREQ = 1: Higher PWM frequency (35kHz)
    pwmconf |= 1 << 18;  // PWM_AUTOSCALE = 1: Enable autoscaling
    pwmconf |= 1 << 19;  // PWM_AUTOGRAD = 1: Enable autogradient
    pwmconf |= 0 << 24;  // FREEWHEEL = 0: Normal operation
    writeRegister(REG_PWMCONF, pwmconf);

    // 7. Configure CoolStep
    uint32_t coolconf = 0;
    coolconf |= 1 << 0;   // SEMIN = 1: Enable CoolStep
    coolconf |= 1 << 5;   // SEUP = 1: Current increment steps
    coolconf |= 2 << 8;   // SEMAX = 2: Upper CoolStep threshold
    coolconf |= 0 << 13;  // SEDN = 0: Slow reaction to lower threshold
    coolconf |= 0 << 16;  // SEIMIN = 0: Minimum current = 1/2
    writeRegister(REG_COOLCONF, coolconf);

    // 8. Configure StallGuard more sensitively
    uint32_t sgconf = 0;
    sgconf |= 4 << 0;   // CS = 4: Slightly reduced current scale
    sgconf |= 2 << 8;   // SGT = 2: Higher sensitivity
    sgconf |= 1 << 16;  // SFILT = 1: Enable filter for smoother response
    writeRegister(REG_SGCONF, sgconf);

    // 9. Configure motion parameters for smooth movement
    writeRegister(REG_RAMPMODE, 0);   // Position mode
    writeRegister(REG_VSTART, 1);     // Start velocity
    writeRegister(REG_A1, 1000);      // First acceleration - moderate
    writeRegister(REG_V1, 50000);     // First velocity - moderate
    writeRegister(REG_AMAX, 2000);    // Maximum acceleration - moderate
    writeRegister(REG_VMAX, 100000);  // Maximum velocity - moderate
    writeRegister(REG_DMAX, 2000);    // Maximum deceleration - moderate
    writeRegister(REG_D1, 1000);      // First deceleration - moderate
    writeRegister(REG_VSTOP, 10);     // Stop velocity - very slow

    // 10. Re-enable the driver
    digitalWrite(EN_PIN, LOW);
    delay(100);

    Serial.println("Driver reset and reconfigured successfully");
    Serial.println("Running diagnostics to verify configuration...");
    runDiagnostics();
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
    Serial.println(clockwise ? "\nMoving motor CW one revolution..."
                             : "\nMoving motor CCW one revolution...");

    // First, verify driver status
    uint32_t drvstatus = readRegister(REG_DRVSTATUS);
    Serial.print("Initial driver status: 0x");
    Serial.println(drvstatus, HEX);

    // Check if driver is enabled
    if (digitalRead(EN_PIN) == HIGH) {
        Serial.println("Driver was disabled - Enabling now");
        digitalWrite(EN_PIN, LOW);
        delay(10);
    }

    // Verify current settings before movement
    uint32_t ihold_irun = readRegister(REG_IHOLD_IRUN);
    float    irun       = ((ihold_irun >> 8) & 0x1F) * 0.31;
    Serial.print("Run current before move: ");
    Serial.print(irun, 2);
    Serial.println("A");

    // Set direction
    digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);
    delayMicroseconds(100);  // Setup time for direction change

    // For a 1.8° motor with 32 microsteps
    const int STEPS_PER_REV = 200 * 32;  // 6400 microsteps per revolution

    // Moderate initial speed with gradual acceleration
    int       step_high_time = 100;  // Start at 100µs
    int       step_low_time  = 100;  // Equal high/low time
    const int MIN_STEP_TIME  = 20;   // Maximum speed limit (25kHz)
    const int ACCEL_STEPS    = 400;  // Steps over which to accelerate

    Serial.println("Starting movement...");

    // Move the motor with smooth acceleration
    for (int i = 0; i < STEPS_PER_REV; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(step_high_time);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(step_low_time);

        // Gradually decrease step time (accelerate) over first ACCEL_STEPS
        if (i < ACCEL_STEPS && step_high_time > MIN_STEP_TIME) {
            step_high_time = 100 - (80 * i / ACCEL_STEPS);
            step_low_time  = step_high_time;
        }

        // Print progress and check status periodically
        if (i % 800 == 0) {  // Check every 800 steps (1/8 revolution)
            drvstatus = readRegister(REG_DRVSTATUS);
            Serial.print("Step ");
            Serial.print(i);
            Serial.print("/");
            Serial.print(STEPS_PER_REV);
            Serial.print(" - Status: 0x");
            Serial.println(drvstatus, HEX);

            if ((drvstatus & 0xF0000000) != 0) {
                Serial.println("WARNING: Driver reporting error during movement!");
                if (drvstatus & (1UL << 31))
                    Serial.println("- Motor open load B");
                if (drvstatus & (1UL << 30))
                    Serial.println("- Motor open load A");
                if (drvstatus & (1UL << 29))
                    Serial.println("- Short to GND B");
                if (drvstatus & (1UL << 28))
                    Serial.println("- Short to GND A");
            }
        }
    }

    // Final status check
    drvstatus = readRegister(REG_DRVSTATUS);
    Serial.print("\nFinal driver status: 0x");
    Serial.println(drvstatus, HEX);

    Serial.println("Movement complete");
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

// Function to monitor SPI communication for a register
void monitorSPITransaction(uint8_t reg) {
    Serial.println("\n=== SPI Transaction Monitor ===");
    Serial.print("Register 0x");
    Serial.print(reg, HEX);
    Serial.print(" (");

    // Print register name
    switch (reg) {
        case REG_GCONF:
            Serial.print("GCONF");
            break;
        case REG_GSTAT:
            Serial.print("GSTAT");
            break;
        case REG_IHOLD_IRUN:
            Serial.print("IHOLD_IRUN");
            break;
        case REG_CHOPCONF:
            Serial.print("CHOPCONF");
            break;
        case REG_DRVSTATUS:
            Serial.print("DRVSTATUS");
            break;
        case REG_COOLCONF:
            Serial.print("COOLCONF");
            break;
        case REG_PWMCONF:
            Serial.print("PWMCONF");
            break;
        default:
            Serial.print("Unknown");
            break;
    }
    Serial.println(")");

    // Read operation
    uint32_t data = 0;
    digitalWrite(CS_PIN, LOW);

    // Send read command
    uint8_t cmd = reg & 0x7F;  // Read operation (MSB=0)
    Serial.print("TX: 0x");
    Serial.println(cmd, HEX);
    SPI.transfer(cmd);

    // Read 4 bytes
    Serial.print("RX: 0x");
    for (int i = 0; i < 4; i++) {
        uint8_t received = SPI.transfer(0x00);
        data |= ((uint32_t)received << (24 - (i * 8)));
        Serial.print(received, HEX);
        Serial.print(" ");
    }
    Serial.println();

    digitalWrite(CS_PIN, HIGH);

    // Print decoded value
    Serial.print("Decoded value: 0x");
    Serial.println(data, HEX);
    Serial.println("===========================\n");
}

// Function to verify SPI communication with the driver
bool verifySPICommunication() {
    Serial.println("\n=== Testing SPI Communication ===");
    bool testPassed = true;

    // Test 1: Write and read back GCONF register
    Serial.println("\nTest 1: GCONF Register R/W Test");
    uint32_t testValue = 0x00000004;  // Basic configuration value
    writeRegister(REG_GCONF, testValue);
    uint32_t readValue = readRegister(REG_GCONF);
    Serial.print("Written: 0x");
    Serial.print(testValue, HEX);
    Serial.print(" Read: 0x");
    Serial.println(readValue, HEX);
    if (readValue != testValue) {
        Serial.println("FAILED: GCONF mismatch");
        testPassed = false;
    }

    // Test 2: Clear and verify GSTAT register
    Serial.println("\nTest 2: GSTAT Register Clear Test");
    writeRegister(REG_GSTAT, 0x07);  // Clear all flags
    readValue = readRegister(REG_GSTAT);
    Serial.print("GSTAT after clear: 0x");
    Serial.println(readValue, HEX);
    if (readValue & 0x07) {
        Serial.println("WARNING: GSTAT flags still set after clear");
    }

    // Test 3: CHOPCONF register test
    Serial.println("\nTest 3: CHOPCONF Register R/W Test");
    testValue = 0x10000053;  // Test configuration
    writeRegister(REG_CHOPCONF, testValue);
    readValue = readRegister(REG_CHOPCONF);
    Serial.print("Written: 0x");
    Serial.print(testValue, HEX);
    Serial.print(" Read: 0x");
    Serial.println(readValue, HEX);
    if (readValue != testValue) {
        Serial.println("FAILED: CHOPCONF mismatch");
        testPassed = false;
    }

    // Test 4: Read-only register test (DRVSTATUS)
    Serial.println("\nTest 4: DRVSTATUS Register Read Test");
    readValue = readRegister(REG_DRVSTATUS);
    Serial.print("DRVSTATUS: 0x");
    Serial.println(readValue, HEX);
    if (readValue == 0xFFFFFFFF || readValue == 0x00000000) {
        Serial.println("WARNING: Suspicious DRVSTATUS value");
        testPassed = false;
    }

    // Test 5: Current setting test
    Serial.println("\nTest 5: Current Setting Test");
    testValue = 0x00020808;  // Example current setting
    writeRegister(REG_IHOLD_IRUN, testValue);
    readValue = readRegister(REG_IHOLD_IRUN);
    Serial.print("Written: 0x");
    Serial.print(testValue, HEX);
    Serial.print(" Read: 0x");
    Serial.println(readValue, HEX);
    if (readValue != testValue) {
        Serial.println("FAILED: IHOLD_IRUN mismatch");
        testPassed = false;
    }

    // Final status
    Serial.println("\n=== SPI Communication Test Results ===");
    if (testPassed) {
        Serial.println("All tests PASSED - SPI communication verified");
    } else {
        Serial.println("Some tests FAILED - Check SPI connections and settings");
        Serial.println("Verify:");
        Serial.println("1. CS_PIN connection (Pin 5)");
        Serial.println("2. MOSI connection (Pin 23)");
        Serial.println("3. MISO connection (Pin 19)");
        Serial.println("4. SCK connection (Pin 18)");
        Serial.println("5. Power supply to driver");
    }
    Serial.println("===================================\n");

    return testPassed;
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
    Serial.println("  'm' - Monitor SPI communication");
    Serial.println("  'v' - Verify SPI communication");
    Serial.println("  'h' - Show this menu");
    Serial.println("=======================================");
}

// Main loop
void loop() {
    if (Serial.available() > 0) {
        char command = Serial.read();

        switch (command) {
            case '+':  // Increase current by ~10%
                adjustMotorCurrent(true);
                break;

            case '-':  // Decrease current by ~10%
                adjustMotorCurrent(false);
                break;

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

            case 'm':  // Monitor SPI communication
            case 'M':
                // Monitor a few important registers
                monitorSPITransaction(REG_GCONF);
                monitorSPITransaction(REG_GSTAT);
                monitorSPITransaction(REG_IHOLD_IRUN);
                monitorSPITransaction(REG_DRVSTATUS);
                break;

            case 'v':  // Verify SPI communication
            case 'V':
                verifySPICommunication();
                break;

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
                Serial.println("  'm' - Monitor SPI communication");
                Serial.println("  'v' - Verify SPI communication");
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