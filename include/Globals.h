#ifndef PWM_ENCODER_INSTANCES_H
#define PWM_ENCODER_INSTANCES_H

#include "Encoders/EncoderConfig.h"
#include "Encoders/EncoderFactory.h"
#include "Encoders/EncoderInterface.h"

extern MotionSystem::EncoderInterface* pwmEncoders[MotionSystem::Config::TMC5160T_Driver::NUM_PWM_ENCODERS];
void                                   initializePWMEncoders();

extern MotionSystem::EncoderInterface* diffEncoder;
void                                   initializeDiffEncoders();

#endif  // PWM_ENCODER_INSTANCES_H

#ifndef MOTOR_INSTANCES_H
#define MOTOR_INSTANCES_H

#include "Motors/TmcController.h"

extern MotionSystem::TmcController motors[MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS];
void                               initializeMotors();

#endif  // MOTOR_INSTANCES_H