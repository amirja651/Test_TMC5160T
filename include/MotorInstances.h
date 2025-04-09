#ifndef MOTOR_INSTANCES_H
#define MOTOR_INSTANCES_H

#include "MotorController.h"

extern MotionSystem::MotorController motors[MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS];
void                                 initializeMotors();

#endif  // MOTOR_INSTANCES_H