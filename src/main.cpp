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
#include "CommandHandler.h"
#include "Config.h"
#include "Encoders/EncoderConfig.h"
#include "Encoders/EncoderFactory.h"
#include "LimitSwitch.h"
#include "MotionController.h"
#include "PIDController.h"
#include "StatusReporter.h"
#include "StepperMotor.h"
#include "Types.h"

using namespace MotionSystem::Types;

// Create encoder configurations
MotionSystem::EncoderConfig diffEncoderConfig = MotionSystem::EncoderFactory::createDifferentialConfig(
    MotionSystem::Config::Pins::ENCODER_A_PIN, MotionSystem::Config::Pins::ENCODER_B_PIN, 0);

MotionSystem::EncoderConfig pwmEncoderConfig =
    MotionSystem::EncoderFactory::createPWMConfig(MotionSystem::Config::Pins::ENCODER_P_PIN,  // signal pin
                                                  MotionSystem::Config::Pins::ENCODER_P_PIN   // interrupt pin
    );

// Create encoders using factory
MotionSystem::EncoderInterface* diffEncoder = MotionSystem::EncoderFactory::createEncoder(
    MotionSystem::EncoderFactory::EncoderType::DIFFERENTIAL, diffEncoderConfig);

MotionSystem::EncoderInterface* pwmEncoder =
    MotionSystem::EncoderFactory::createEncoder(MotionSystem::EncoderFactory::EncoderType::PWM, pwmEncoderConfig);

MotionSystem::StepperMotor     motor;
MotionSystem::LimitSwitch      limitSwitch;
MotionSystem::PIDController    pidController(diffEncoder);
MotionSystem::StatusReporter   statusReporter(diffEncoder, &pidController, &limitSwitch);
MotionSystem::MotionController motionController(diffEncoder, &motor, &pidController, &limitSwitch, &statusReporter);
TaskHandle_t                   serialTaskHandle       = NULL;
TaskHandle_t                   commandTaskHandle      = NULL;
TaskHandle_t                   motorUpdateTaskHandle0 = NULL;
TaskHandle_t                   motorUpdateTaskHandle1 = NULL;
TaskHandle_t                   motorUpdateTaskHandle2 = NULL;
TaskHandle_t                   motorUpdateTaskHandle3 = NULL;
QueueHandle_t                  commandQueue;

#define COMMAND_BUFFER_SIZE 32

void serialTask(void* pvParameters)
{
    char    inputBuffer[COMMAND_BUFFER_SIZE];
    uint8_t bufferIndex = 0;
    while (1)
    {
        if (Serial.available() > 0)
        {
            char c = Serial.read();
            if (c == '\n' || c == '\r')
            {
                if (bufferIndex > 0)
                {
                    inputBuffer[bufferIndex] = '\0';
                    if (strcmp(inputBuffer, "h") == 0 || strcmp(inputBuffer, "?") == 0)
                    {
                        MotionSystem::CommandHandler::getInstance().printCommandGuide();
                        bufferIndex = 0;
                        continue;
                    }

                    if (strncmp(inputBuffer, "m ", 2) == 0)
                    {
                        if (strlen(inputBuffer) < 5)  // Changed from 9 to 5 (m 1 w = 5 chars)
                        {
                            Serial.println(F("❌ Invalid command. Use h/? for help"));
                            bufferIndex = 0;
                            continue;
                        }

                        int  motorNum = inputBuffer[2] - '0';
                        char cmd      = inputBuffer[4];
                        if (motorNum < 1 || motorNum > MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS)
                        {
                            Serial.print(F("❌ Invalid motor number (1-"));
                            Serial.print(MotionSystem::Config::TMC5160T_Driver::NUM_MOTORS);
                            Serial.println(F(")"));
                            bufferIndex = 0;
                            continue;
                        }

                        if (!MotionSystem::CommandHandler::getInstance().isValidMotorCommand(cmd))
                        {
                            Serial.println(F("❌ Invalid command. Use h/? for help"));
                            bufferIndex = 0;
                            continue;
                        }
                    }

                    if (xQueueSend(commandQueue, inputBuffer, portMAX_DELAY) != pdPASS)
                    {
                        Serial.println(F("❌ Failed to send command to queue"));
                    }

                    bufferIndex = 0;
                }
            }

            else if (bufferIndex < COMMAND_BUFFER_SIZE - 1)
            {
                inputBuffer[bufferIndex++] = c;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void commandTask(void* pvParameters)
{
    char inputBuffer[COMMAND_BUFFER_SIZE];
    while (1)
    {
        if (xQueueReceive(commandQueue, inputBuffer, portMAX_DELAY) == pdPASS)
        {
            Serial.print(F("Command received: "));
            Serial.println(inputBuffer);
            Serial.println();
            if (strncmp(inputBuffer, "m ", 2) == 0)
            {
                int  motorNum = inputBuffer[2] - '0';
                char cmd      = inputBuffer[4];
                MotionSystem::CommandHandler::getInstance().processCommand(cmd, motorNum);
            }

            else
            {
                Serial.println(F("❌ Invalid command format. Enter h/? for help"));
            }
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

    // Initialize encoders
    diffEncoder->begin();
    pwmEncoder->begin();

    motionController.init();
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

    initializeMotors();
    MotionSystem::CommandHandler::getInstance().printCommandGuide();
    xTaskCreate(serialTask, "SerialTask", 4096, NULL, 2, &serialTaskHandle);
    xTaskCreate(commandTask, "CommandTask", 4096, NULL, 2, &commandTaskHandle);
    xTaskCreate(motorUpdateTask0, "MotorUpdateTask0", 4096, NULL, 3, &motorUpdateTaskHandle0);
    xTaskCreate(motorUpdateTask1, "MotorUpdateTask1", 4096, NULL, 3, &motorUpdateTaskHandle1);
    xTaskCreate(motorUpdateTask2, "MotorUpdateTask2", 4096, NULL, 3, &motorUpdateTaskHandle2);
    xTaskCreate(motorUpdateTask3, "MotorUpdateTask3", 4096, NULL, 3, &motorUpdateTaskHandle3);
    vTaskStartScheduler();
}

void loop()
{
    motionController.processCommands();

    // Read and display PWM encoder position
    MotionSystem::Types::EncoderPosition pwmPosition = pwmEncoder->readPosition();
    MotionSystem::Types::MicronPosition  pwmMicrons  = pwmEncoder->countsToMicrons(pwmPosition);

    Serial.print(F("PWM Encoder Position: "));
    Serial.print(pwmPosition);
    Serial.print(F(" counts ("));
    Serial.print(pwmMicrons, 2);
    Serial.println(F(" μm)"));

    delay(10);
}