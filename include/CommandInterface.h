#ifndef COMMAND_INTERFACE_H
#define COMMAND_INTERFACE_H

#include <Arduino.h>
#include "config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define COMMAND_TASK_STACK_SIZE 4096
#define COMMAND_TASK_PRIORITY   1
#define COMMAND_QUEUE_SIZE      10
#define COMMAND_BUFFER_SIZE     256

class CommandInterface {
public:
    CommandInterface();
    ~CommandInterface();

    // Initialize the command interface
    void begin();

    // Task handle for the command processing task
    TaskHandle_t commandTaskHandle;

private:
    // Queue for receiving commands
    QueueHandle_t commandQueue;

    // Command processing task
    static void commandTask(void* parameter);

    // Process received command
    void processCommand(const char* command);

    // Display pin status
    void displayPinStatus();

    // Display command menu
    void displayCommandMenu();

    // Test SPI communication for a specific driver
    bool testSPICommunication(uint8_t driverNumber);

    // Buffer for receiving commands
    char commandBuffer[COMMAND_BUFFER_SIZE];
};

#endif  // COMMAND_INTERFACE_H