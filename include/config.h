// SPI Pins for ESP32
#define SPI_MOSI   23
#define SPI_MISO   19
#define SPI_SCK    18
#define TMC_CS_PIN 5

// Motor control pins
#define STEP_PIN 4
#define DIR_PIN  2
#define EN_PIN   15

// Motor driver config
#define TMC_CURRENT_MA 1000  // Motor RMS current in mA
#define MICROSTEPS     16    // Microstep resolution
