#ifndef TASKS_H
#define TASKS_H

#include <stdint.h>

namespace MotionSystem
{
    namespace Tasks
    {
        constexpr uint16_t PID_TASK_STACK_SIZE = 4096;
        constexpr uint8_t  PID_TASK_PRIORITY   = 3;
        constexpr uint8_t  PID_TASK_CORE       = 1;

        constexpr uint16_t MOTION_TASK_STACK_SIZE = 4096;
        constexpr uint8_t  MOTION_TASK_PRIORITY   = 2;
        constexpr uint8_t  MOTION_TASK_CORE       = 1;

        constexpr uint16_t STATUS_TASK_STACK_SIZE = 4096;
        constexpr uint8_t  STATUS_TASK_PRIORITY   = 1;
        constexpr uint8_t  STATUS_TASK_CORE       = 0;
    }  // namespace Tasks

}  // namespace MotionSystem

#endif  // TASKS_H