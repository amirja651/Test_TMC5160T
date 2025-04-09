#include "Encoders\EncoderInstances.h"
#include "MotorControllers\MotorInstances.h"

MotionSystem::TmcController motors[MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS] = {
    MotionSystem::TmcController(
        "Motor 1", MotionSystem::Config::SPI::MOTOR1_CS, MotionSystem::Config::TMC5160T_Driver::MOTOR1_STEP_PIN,
        MotionSystem::Config::TMC5160T_Driver::MOTOR1_DIR_PIN, MotionSystem::Config::TMC5160T_Driver::MOTOR1_EN_PIN),
    MotionSystem::TmcController(
        "Motor 2", MotionSystem::Config::SPI::MOTOR2_CS, MotionSystem::Config::TMC5160T_Driver::MOTOR2_STEP_PIN,
        MotionSystem::Config::TMC5160T_Driver::MOTOR2_DIR_PIN, MotionSystem::Config::TMC5160T_Driver::MOTOR2_EN_PIN),
    MotionSystem::TmcController(
        "Motor 3", MotionSystem::Config::SPI::MOTOR3_CS, MotionSystem::Config::TMC5160T_Driver::MOTOR3_STEP_PIN,
        MotionSystem::Config::TMC5160T_Driver::MOTOR3_DIR_PIN, MotionSystem::Config::TMC5160T_Driver::MOTOR3_EN_PIN),
    MotionSystem::TmcController(
        "Motor 4", MotionSystem::Config::SPI::MOTOR4_CS, MotionSystem::Config::TMC5160T_Driver::MOTOR4_STEP_PIN,
        MotionSystem::Config::TMC5160T_Driver::MOTOR4_DIR_PIN, MotionSystem::Config::TMC5160T_Driver::MOTOR4_EN_PIN)};

void initializeMotors()
{
    for (uint8_t i = 0; i < MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS; i++)
    {
        motors[i].begin();
    }
}

MotionSystem::EncoderConfig pwmEncoderConfig[MotionSystem::Config::TMC5160T_Driver::NUM_PWM_ENCODERS] = {
    MotionSystem::EncoderFactory::createPWMConfig(MotionSystem::Config::Pins::ENCODER_P1_PIN,  // signal pin
                                                  MotionSystem::Config::Pins::ENCODER_P1_PIN   // interrupt pin
                                                  ),
    MotionSystem::EncoderFactory::createPWMConfig(MotionSystem::Config::Pins::ENCODER_P2_PIN,  // signal pin
                                                  MotionSystem::Config::Pins::ENCODER_P2_PIN   // interrupt pin
                                                  ),
    MotionSystem::EncoderFactory::createPWMConfig(MotionSystem::Config::Pins::ENCODER_P3_PIN,  // signal pin
                                                  MotionSystem::Config::Pins::ENCODER_P3_PIN   // interrupt pin
                                                  ),
    MotionSystem::EncoderFactory::createPWMConfig(MotionSystem::Config::Pins::ENCODER_P4_PIN,  // signal pin
                                                  MotionSystem::Config::Pins::ENCODER_P4_PIN   // interrupt pin
                                                  )};

MotionSystem::EncoderInterface* pwmEncoders[MotionSystem::Config::TMC5160T_Driver::NUM_PWM_ENCODERS] = {
    MotionSystem::EncoderFactory::createEncoder(MotionSystem::EncoderFactory::EncoderType::PWM, pwmEncoderConfig[0]),
    MotionSystem::EncoderFactory::createEncoder(MotionSystem::EncoderFactory::EncoderType::PWM, pwmEncoderConfig[1]),
    MotionSystem::EncoderFactory::createEncoder(MotionSystem::EncoderFactory::EncoderType::PWM, pwmEncoderConfig[2]),
    MotionSystem::EncoderFactory::createEncoder(MotionSystem::EncoderFactory::EncoderType::PWM, pwmEncoderConfig[3])};

void initializePWMEncoders()
{
    for (uint8_t i = 0; i < MotionSystem::Config::TMC5160T_Driver::NUM_PWM_ENCODERS; i++)
    {
        pwmEncoders[i]->begin();
    }
}

// Create encoder configurations
MotionSystem::EncoderConfig diffEncoderConfig = MotionSystem::EncoderFactory::createDifferentialConfig(
    MotionSystem::Config::Pins::ENCODER_A_PIN, MotionSystem::Config::Pins::ENCODER_B_PIN, 0);

// Create encoders using factory
MotionSystem::EncoderInterface* diffEncoder = MotionSystem::EncoderFactory::createEncoder(
    MotionSystem::EncoderFactory::EncoderType::DIFFERENTIAL, diffEncoderConfig);

void initializeDiffEncoders()
{
    diffEncoder->begin();
}