#ifndef MOTION_GLOBALS_H
#define MOTION_GLOBALS_H

#include "Globals.h"
#include "Helper/Logger.h"
#include "Helper/StatusReporter.h"
#include "Motion/LimitSwitch.h"
#include "Motion/MotionController.h"
#include "Motion/PIDController.h"

// Global instances for motion control system
extern MotionSystem::LimitSwitch      limitSwitch;
extern MotionSystem::PIDController    pidController[MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS];
extern MotionSystem::StatusReporter   statusReporter;
extern MotionSystem::MotionController motionController[MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS];

// Initialization function
void initializeMotionSystem();

#endif  // MOTION_GLOBALS_H