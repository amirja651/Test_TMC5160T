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

/*
// Create encoder configurations
EncoderConfig diffEncoderConfig = EncoderFactory::createDifferentialConfig(
    Config::Pins::ENCODER_A_PIN, Config::Pins::ENCODER_B_PIN, 0);

// Create encoders using factory
EncoderInterface* diffEncoder = EncoderFactory::createEncoder(
    EncoderFactory::EncoderType::DIFFERENTIAL, diffEncoderConfig);

void initializeDiffEncoders()
{
    diffEncoder->begin();
}
*/

// Define global instances
LimitSwitch limitSwitch;

PIDController pidController[System::NUM_MOTORS] = {PIDController(pwmEncoders[0]), PIDController(pwmEncoders[1]),
                                                   PIDController(pwmEncoders[2]), PIDController(pwmEncoders[3])};

MotionController motionController[System::NUM_MOTORS] = {
    MotionController(pwmEncoders[0], &motors[0], &pidController[0], &limitSwitch),
    MotionController(pwmEncoders[1], &motors[1], &pidController[1], &limitSwitch),
    MotionController(pwmEncoders[2], &motors[2], &pidController[2], &limitSwitch),
    MotionController(pwmEncoders[3], &motors[3], &pidController[3], &limitSwitch)};

void initializeMotors()
{
    for (uint8_t i = 0; i < System::NUM_MOTORS; i++)
    {
        motors[i].begin();
    }
}

void initializePWMEncoders()
{
    for (uint8_t i = 0; i < System::NUM_PWM_ENCODERS; i++)
    {
        pwmEncoders[i]->begin();
    }
}

void initializeMotionSystem()
{
    limitSwitch.init();

    for (uint8_t i = 0; i < System::NUM_MOTORS; i++)
    {
        // Initialize components in the correct order
        pidController[i].init();
        motionController[i].begin();
        motionController[i].startTask();

        Logger::getInstance().log(F("Motion system initialized for motor "));
        Logger::getInstance().logln(String(i + 1));
    }
}

auto& commandHandler = CommandHandler::getInstance();

void setup()
{
    commandHandler.beginSerial();

    Logger::getInstance().begin();

    Logger::getInstance().logln(
        F("\n ==================== Initializing High Precision Motion Control System ===================="));

    initializeMotors();
    initializePWMEncoders();
    initializeMotionSystem();

    CommandHandler::getInstance().printCommandGuide();
}

void loop()
{
    delay(10);
}