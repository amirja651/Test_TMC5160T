#include <Arduino.h>

#include "Encoders/EncoderFactory.h"
#include "Encoders/PWMEncoder.h"
#include "Helper/Logger.h"

using namespace MotionSystem;

// Create encoder configuration
static EncoderConfig pwmEncoderConfigTest = EncoderFactory::createPWMConfig(35, 35);

// Create encoder instance
PWMEncoder encoderTest(pwmEncoderConfigTest);

void setup2()
{
    Serial.begin(115200);
    delay(1000);  // Wait for serial to initialize

    // Initialize the logger
    Logger::getInstance().begin();
    Logger::getInstance().log(F("PWM Encoder Example Started"));

    // Initialize the encoder
    encoderTest.begin();
    encoderTest.resetPosition();

    Logger::getInstance().log(F("Encoder initialized and ready"));
}

void loop2()
{
    // Read the current position
    Types::EncoderPosition position = encoderTest.readPosition();

    // Convert position to microns (assuming 0.5mm lead screw pitch)
    float microns = (position / 1000.0f) * 500.0f;  // 1000 counts/rev * 0.5mm pitch

    // Log the position
    Logger::getInstance().logf(F("Encoder Position: %ld counts (%.2f μm)"), position, microns);

    delay(100);  // Update every 100ms
}