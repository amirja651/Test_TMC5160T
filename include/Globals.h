#ifndef GLOBAL_INSTANCES_H
#define GLOBAL_INSTANCES_H

#include "Encoders/EncoderConfig.h"
#include "Encoders/EncoderFactory.h"
#include "Encoders/EncoderInterface.h"
#include "Motors/TmcController.h"

extern MotionSystem::TmcController     motors[MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS];
extern MotionSystem::EncoderInterface* pwmEncoders[MotionSystem::Config::TMC5160T_Driver::NUM_PWM_ENCODERS];
extern MotionSystem::EncoderInterface* diffEncoder;

void initializeMotors();
void initializePWMEncoders();
void initializeDiffEncoders();

#endif  // GLOBAL_INSTANCES_H