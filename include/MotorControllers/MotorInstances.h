#ifndef MOTOR_INSTANCES_H
#define MOTOR_INSTANCES_H

#include "MotorControllers\TmcController.h"

extern MotionSystem::TmcController motors[MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS];
void                               initializeMotors();

#endif  // MOTOR_INSTANCES_H