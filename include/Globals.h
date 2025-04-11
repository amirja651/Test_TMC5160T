#ifndef GLOBAL_INSTANCES_H
#define GLOBAL_INSTANCES_H

#include "Encoders/EncoderConfig.h"
#include "Encoders/EncoderFactory.h"
#include "Encoders/EncoderInterface.h"
#include "Helper/Logger.h"
#include "Helper/System.h"
#include "Motion/LimitSwitch.h"
#include "Motion/MotionController.h"
#include "Motion/PIDController.h"
#include "Motors/TmcController.h"

extern MotionSystem::TmcController     motors[MotionSystem::System::NUM_MOTORS];
extern MotionSystem::EncoderInterface* pwmEncoders[MotionSystem::System::NUM_PWM_ENCODERS];
extern MotionSystem::EncoderInterface* diffEncoder;
extern MotionSystem::LimitSwitch       limitSwitch;
extern MotionSystem::PIDController     pidController[MotionSystem::System::NUM_MOTORS];
extern MotionSystem::MotionController  motionController[MotionSystem::System::NUM_MOTORS];

void initializeMotors();
void initializePWMEncoders();
void initializeDiffEncoders();
void initializeMotionSystem();

#endif  // GLOBAL_INSTANCES_H