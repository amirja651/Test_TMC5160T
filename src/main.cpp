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
#include "ESP32Encoder.h"
#include "LimitSwitch.h"
#include "MAE3Encoder.h"
#include "MotionController.h"
#include "PIDController.h"
#include "StatusReporter.h"
#include "StepperMotor.h"
#include "Types.h"

// Create instances of system components
MotionSystem::MAE3Encoder      encoder2(35);
MotionSystem::ESP32Encoder     encoder;
MotionSystem::StepperMotor     motor;
MotionSystem::LimitSwitch      limitSwitch;
MotionSystem::PIDController    pidController(&encoder);
MotionSystem::StatusReporter   statusReporter(&encoder, &pidController, &limitSwitch);
MotionSystem::MotionController motionController(&encoder, &motor, &pidController, &limitSwitch, &statusReporter);

// Task handles
TaskHandle_t serialTaskHandle       = NULL;
TaskHandle_t commandTaskHandle      = NULL;
TaskHandle_t motorUpdateTaskHandle0 = NULL;
TaskHandle_t motorUpdateTaskHandle1 = NULL;
TaskHandle_t motorUpdateTaskHandle2 = NULL;
TaskHandle_t motorUpdateTaskHandle3 = NULL;

// Queue for serial commands
QueueHandle_t commandQueue;

// Command buffer size
#define COMMAND_BUFFER_SIZE 32

// Task for handling serial commands
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

                    // Check for help command
                    if (strcmp(inputBuffer, "h") == 0 || strcmp(inputBuffer, "?") == 0)
                    {
                        MotionSystem::CommandHandler::getInstance().printCommandGuide();
                        bufferIndex = 0;
                        continue;
                    }

                    // Validate command format
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

                    // Send command to queue
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

// Task for processing commands
void commandTask(void* pvParameters)
{
    char inputBuffer[COMMAND_BUFFER_SIZE];

    while (1)
    {
        if (xQueueReceive(commandQueue, inputBuffer, portMAX_DELAY) == pdPASS)
        {
            // Echo the command back to the user
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

// Task for updating motor states
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

// Task for updating motor states
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

// Task for updating motor states
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

// Task for updating motor states
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

/**
 * @brief Initial setup of the system
 * Configures serial communication, SPI interface, and motor controller
 */
void setup()
{
    // Initialize serial communication
    Serial.begin(MotionSystem::Config::System::SERIAL_BAUD_RATE);
    delay(MotionSystem::Config::System::STARTUP_DELAY_MS);
    while (!Serial)
    {
        delay(10);
    }

    Serial.println(F("\n ==================== Initializing High Precision Motion Control System ===================="));

    motionController.init();
    pidController.startTask();
    motionController.startTask();
    statusReporter.startTask();

    // Create command queue with proper size
    commandQueue = xQueueCreate(10, COMMAND_BUFFER_SIZE);
    if (commandQueue == NULL)
    {
        Serial.println(F("Failed to create command queue ❌"));
        while (1)
            ;
    }

    // Initialize all motor controllers
    initializeMotors();

    MotionSystem::CommandHandler::getInstance().printCommandGuide();

    // Create tasks with increased stack sizes
    xTaskCreate(serialTask, "SerialTask", 4096, NULL, 2, &serialTaskHandle);
    xTaskCreate(commandTask, "CommandTask", 4096, NULL, 2, &commandTaskHandle);
    xTaskCreate(motorUpdateTask0, "MotorUpdateTask0", 4096, NULL, 3, &motorUpdateTaskHandle0);
    xTaskCreate(motorUpdateTask1, "MotorUpdateTask1", 4096, NULL, 3, &motorUpdateTaskHandle1);
    xTaskCreate(motorUpdateTask2, "MotorUpdateTask2", 4096, NULL, 3, &motorUpdateTaskHandle2);
    xTaskCreate(motorUpdateTask3, "MotorUpdateTask3", 4096, NULL, 3, &motorUpdateTaskHandle3);

    // Start scheduler
    vTaskStartScheduler();

    // Initialize encoder
    encoder2.begin();

    // Allow time for tasks to start
    delay(100);
}

void loop()
{
    // Process commands from serial port
    motionController.processCommands();

    // Update encoder position
    if (encoder2.update())
    {
        // Print position only when it changes
        Serial.print(F("Position: "));
        Serial.print(encoder2.getPositionDegrees(), 2);
        Serial.print(F("° (Pulse width: "));
        Serial.print(encoder2.getPulseWidth());
        Serial.println(F(" μs)"));
    }

    // Small delay to prevent overwhelming the serial output
    // Main loop runs at lower priority
    delay(10);
}