#include "Globals.h"
#include "Helper/CommandHandler.h"
#include "Helper/Logger.h"
#include "Helper/Pins.h"
#include "Helper/System.h"

using namespace MotionSystem;

TmcController motors[System::NUM_MOTORS] = {
    TmcController("Motor 1", Pins::SPI::MOTOR1_CS, Pins::TMC5160T_Driver::MOTOR1_STEP_PIN,
                  Pins::TMC5160T_Driver::MOTOR1_DIR_PIN, Pins::TMC5160T_Driver::MOTOR1_EN_PIN,
                  MotorType::P28SHD4611_12SK),
    TmcController("Motor 2", Pins::SPI::MOTOR2_CS, Pins::TMC5160T_Driver::MOTOR2_STEP_PIN,
                  Pins::TMC5160T_Driver::MOTOR2_DIR_PIN, Pins::TMC5160T_Driver::MOTOR2_EN_PIN,
                  MotorType::P28SHD4611_12SK),
    TmcController("Motor 3", Pins::SPI::MOTOR3_CS, Pins::TMC5160T_Driver::MOTOR3_STEP_PIN,
                  Pins::TMC5160T_Driver::MOTOR3_DIR_PIN, Pins::TMC5160T_Driver::MOTOR3_EN_PIN,
                  MotorType::P28SHD4611_12SK),
    TmcController("Motor 4", Pins::SPI::MOTOR4_CS, Pins::TMC5160T_Driver::MOTOR4_STEP_PIN,
                  Pins::TMC5160T_Driver::MOTOR4_DIR_PIN, Pins::TMC5160T_Driver::MOTOR4_EN_PIN,
                  MotorType::P28SHD4611_12SK)};

EncoderConfig pwmEncoderConfig[System::NUM_PWM_ENCODERS] = {
    EncoderFactory::createPWMConfig(Pins::Encoder::PWM::ENCODER_P1_PIN,  // signal pin
                                    Pins::Encoder::PWM::ENCODER_P1_PIN   // interrupt pin
                                    ),
    EncoderFactory::createPWMConfig(Pins::Encoder::PWM::ENCODER_P2_PIN,  // signal pin
                                    Pins::Encoder::PWM::ENCODER_P2_PIN   // interrupt pin
                                    ),
    EncoderFactory::createPWMConfig(Pins::Encoder::PWM::ENCODER_P3_PIN,  // signal pin
                                    Pins::Encoder::PWM::ENCODER_P3_PIN   // interrupt pin
                                    ),
    EncoderFactory::createPWMConfig(Pins::Encoder::PWM::ENCODER_P4_PIN,  // signal pin
                                    Pins::Encoder::PWM::ENCODER_P4_PIN   // interrupt pin
                                    )};

EncoderInterface* pwmEncoders[System::NUM_PWM_ENCODERS] = {
    EncoderFactory::createEncoder(EncoderFactory::EncoderType::PWM, pwmEncoderConfig[0]),
    EncoderFactory::createEncoder(EncoderFactory::EncoderType::PWM, pwmEncoderConfig[1]),
    EncoderFactory::createEncoder(EncoderFactory::EncoderType::PWM, pwmEncoderConfig[2]),
    EncoderFactory::createEncoder(EncoderFactory::EncoderType::PWM, pwmEncoderConfig[3])};

PIDController pidController[System::NUM_MOTORS] = {
    PIDController("PID 1", pwmEncoders[0]), PIDController("PID 2", pwmEncoders[1]),
    PIDController("PID 3", pwmEncoders[2]), PIDController("PID 4", pwmEncoders[3])};

MotionController motionController[System::NUM_MOTORS] = {
    MotionController(pwmEncoders[0], &motors[0], &pidController[0], nullptr),
    MotionController(pwmEncoders[1], &motors[1], &pidController[1], nullptr),
    MotionController(pwmEncoders[2], &motors[2], &pidController[2], nullptr),
    MotionController(pwmEncoders[3], &motors[3], &pidController[3], nullptr)};

void initializeMotionSystem()
{
    // limitSwitch.init();

    for (uint8_t i = 0; i < System::NUM_MOTORS; i++)
    {
        // Initialize components in the correct order
        if (!motionController[i].begin())
        {
            String motorName = "Motor " + String(i + 1) + ": Failed to initialize motion controller";
            Serial.println(motorName);
            continue;
        }
        motionController[i].startTask();
    }
}

auto& commandHandler = CommandHandler::getInstance();

void setup()
{
    Serial.begin(System::SERIAL_BAUD_RATE);
    delay(System::STARTUP_DELAY_MS);
    while (!Serial)
    {
        delay(10);
    }

    Serial.println(F("\n ==================== Initializing High Precision Motion Control System ===================="));

    commandHandler.begin();

    initializeMotionSystem();

    CommandHandler::getInstance().printCommandGuide();
}

void loop()
{
    delay(10);
}