#include "motor_instances.h"

// Define the motor controller instances
MotorController motor1(Config::SPI::MOTOR1_CS, Config::TMC5160T_Driver::MOTOR1_STEP_PIN,
                       Config::TMC5160T_Driver::MOTOR1_DIR_PIN,
                       Config::TMC5160T_Driver::MOTOR1_EN_PIN);

MotorController motor2(Config::SPI::MOTOR2_CS, Config::TMC5160T_Driver::MOTOR2_STEP_PIN,
                       Config::TMC5160T_Driver::MOTOR2_DIR_PIN,
                       Config::TMC5160T_Driver::MOTOR2_EN_PIN);

MotorController motor3(Config::SPI::MOTOR3_CS, Config::TMC5160T_Driver::MOTOR3_STEP_PIN,
                       Config::TMC5160T_Driver::MOTOR3_DIR_PIN,
                       Config::TMC5160T_Driver::MOTOR3_EN_PIN);

MotorController motor4(Config::SPI::MOTOR4_CS, Config::TMC5160T_Driver::MOTOR4_STEP_PIN,
                       Config::TMC5160T_Driver::MOTOR4_DIR_PIN,
                       Config::TMC5160T_Driver::MOTOR4_EN_PIN);