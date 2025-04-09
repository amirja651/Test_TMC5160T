#include "MotorInstances.h"

MotionSystem::MotorController motors[MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS] = {
    MotionSystem::MotorController(
        "Motor 1", MotionSystem::Config::SPI::MOTOR1_CS, MotionSystem::Config::TMC5160T_Driver::MOTOR1_STEP_PIN,
        MotionSystem::Config::TMC5160T_Driver::MOTOR1_DIR_PIN, MotionSystem::Config::TMC5160T_Driver::MOTOR1_EN_PIN),
    MotionSystem::MotorController(
        "Motor 2", MotionSystem::Config::SPI::MOTOR2_CS, MotionSystem::Config::TMC5160T_Driver::MOTOR2_STEP_PIN,
        MotionSystem::Config::TMC5160T_Driver::MOTOR2_DIR_PIN, MotionSystem::Config::TMC5160T_Driver::MOTOR2_EN_PIN),
    MotionSystem::MotorController(
        "Motor 3", MotionSystem::Config::SPI::MOTOR3_CS, MotionSystem::Config::TMC5160T_Driver::MOTOR3_STEP_PIN,
        MotionSystem::Config::TMC5160T_Driver::MOTOR3_DIR_PIN, MotionSystem::Config::TMC5160T_Driver::MOTOR3_EN_PIN),
    MotionSystem::MotorController(
        "Motor 4", MotionSystem::Config::SPI::MOTOR4_CS, MotionSystem::Config::TMC5160T_Driver::MOTOR4_STEP_PIN,
        MotionSystem::Config::TMC5160T_Driver::MOTOR4_DIR_PIN, MotionSystem::Config::TMC5160T_Driver::MOTOR4_EN_PIN)};

void initializeMotors()
{
    for (uint8_t i = 0; i < MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS; i++)
    {
        motors[i].begin();
        Serial.print(F("\nMotor "));
        Serial.print(i + 1);
        Serial.print(F(" - "));
        motors[i].testCommunication();
    }
}