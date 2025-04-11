#include "Motion/MotionGlobals.h"

// Define global instances
MotionSystem::LimitSwitch limitSwitch;

MotionSystem::PIDController pidController[MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS] = {
    MotionSystem::PIDController(pwmEncoders[0]), MotionSystem::PIDController(pwmEncoders[1]),
    MotionSystem::PIDController(pwmEncoders[2]), MotionSystem::PIDController(pwmEncoders[3])};

MotionSystem::StatusReporter statusReporter(diffEncoder, &pidController[0], &limitSwitch);

MotionSystem::MotionController motionController[MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS] = {
    MotionSystem::MotionController(pwmEncoders[0], &motors[0], &pidController[0], &limitSwitch, &statusReporter),
    MotionSystem::MotionController(pwmEncoders[1], &motors[1], &pidController[1], &limitSwitch, &statusReporter),
    MotionSystem::MotionController(pwmEncoders[2], &motors[2], &pidController[2], &limitSwitch, &statusReporter),
    MotionSystem::MotionController(pwmEncoders[3], &motors[3], &pidController[3], &limitSwitch, &statusReporter)};

void initializeMotionSystem()
{
    limitSwitch.init();
    statusReporter.startTask();

    for (uint8_t i = 0; i < MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS; i++)
    {
        // Initialize components in the correct order
        pidController[i].init();
        motionController[i].begin();
        motionController[i].startTask();
        MotionSystem::Logger::getInstance().log(F("Motion system initialized for motor "));
        MotionSystem::Logger::getInstance().logln(String(i + 1));
    }
}