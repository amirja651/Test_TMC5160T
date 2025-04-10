/**
 * High Precision Motion Control for ESP32 with Stepper Motor and Encoder Feedback
 * Components:
 * - Nema 11 Stepper Motor (1.8deg, 200 steps/rev)
 * - Magnetic Encoder (1000PPR/4000CPR)
 * - Stepper Driver with microstepping
 * - ESP32 Controller
 * - Linear Stage with Ultra-Fine-Thread Ball-Point Set Screw
 *
 * Target precision: < 1 micrometer
 */

#include <Arduino.h>

#include "Config.h"
#include "Globals.h"

#include "Helper/CommandHandler.h"
#include "Helper/StatusReporter.h"

#include "Motion/LimitSwitch.h"
#include "Motion/MotionController.h"
#include "Motion/PIDController.h"
#include "Motors/SimpleController.h"

using namespace MotionSystem::Types;

// MotionSystem::SimpleController motor;
MotionSystem::LimitSwitch      limitSwitch;
MotionSystem::PIDController    pidController(pwmEncoders[0]);
MotionSystem::StatusReporter   statusReporter(diffEncoder, &pidController, &limitSwitch);
MotionSystem::MotionController motionController(pwmEncoders[0], &motors[0], &pidController, &limitSwitch,
                                                &statusReporter);
TaskHandle_t                   serialTaskHandle       = NULL;
TaskHandle_t                   motorUpdateTaskHandle0 = NULL;
TaskHandle_t                   motorUpdateTaskHandle1 = NULL;
TaskHandle_t                   motorUpdateTaskHandle2 = NULL;
TaskHandle_t                   motorUpdateTaskHandle3 = NULL;
QueueHandle_t                  commandQueue;

#define COMMAND_BUFFER_SIZE 32

void serialTask(void* pvParameters)
{
    while (1)
    {
        if (Serial.available() > 0)
        {
            String command = Serial.readStringUntil('\n');

            if (command.length() > 10)
            {
                Serial.println(F("❌ Invalid command. Use h/? for help"));
                continue;
            }

            command.trim();
            command.toUpperCase();

            MotionSystem::CommandHandler::getInstance().processCommand(command);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void motorUpdateTask0(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        motors[0].update();
        taskYIELD();  // Allow other tasks to run between motor updates
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void motorUpdateTask1(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        motors[1].update();
        taskYIELD();  // Allow other tasks to run between motor updates
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void motorUpdateTask2(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        motors[2].update();
        taskYIELD();  // Allow other tasks to run between motor updates
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void motorUpdateTask3(void* pvParameters)
{
    const TickType_t xFrequency    = pdMS_TO_TICKS(10);  // Changed from 1ms to 10ms
    TickType_t       xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        motors[3].update();
        taskYIELD();  // Allow other tasks to run between motor updates
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void setup()
{
    Serial.begin(MotionSystem::Config::System::SERIAL_BAUD_RATE);
    delay(MotionSystem::Config::System::STARTUP_DELAY_MS);
    while (!Serial)
    {
        delay(10);
    }

    Serial.println(F("\n ==================== Initializing High Precision Motion Control System ===================="));

    initializeMotors();
    initializePWMEncoders();

    motionController.begin();
    pidController.startTask();
    motionController.startTask();
    statusReporter.startTask();
    commandQueue = xQueueCreate(10, COMMAND_BUFFER_SIZE);
    if (commandQueue == NULL)
    {
        Serial.println(F("Failed to create command queue ❌"));
        while (1)
            ;
    }

    MotionSystem::CommandHandler::getInstance().printCommandGuide();
    xTaskCreate(serialTask, "SerialTask", 4096, NULL, 2, &serialTaskHandle);
    xTaskCreate(motorUpdateTask0, "MotorUpdateTask0", 4096, NULL, 3, &motorUpdateTaskHandle0);
    xTaskCreate(motorUpdateTask1, "MotorUpdateTask1", 4096, NULL, 3, &motorUpdateTaskHandle1);
    xTaskCreate(motorUpdateTask2, "MotorUpdateTask2", 4096, NULL, 3, &motorUpdateTaskHandle2);
    xTaskCreate(motorUpdateTask3, "MotorUpdateTask3", 4096, NULL, 3, &motorUpdateTaskHandle3);
    vTaskStartScheduler();
}

void loop()
{
    // motionController.processCommands();

    // Read and display PWM encoder position
    /** MotionSystem::Types::EncoderPosition pwmPosition = pwmEncoders[0]->readPosition();
    MotionSystem::Types::MicronPosition       pwmMicrons  = pwmEncoders[0]->countsToMicrons(pwmPosition);

    Serial.print(F("PWM Encoder Position: "));
    Serial.print(pwmPosition);
    Serial.print(F(" counts ("));
    Serial.print(pwmMicrons, 2);
    Serial.println(F(" μm)"));
    **/

    delay(10);
}