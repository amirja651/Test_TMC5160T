#include "Helper/Logger.h"
#include <inttypes.h>  // For PRIu32 and other format specifiers
#include "Helper/System.h"

namespace MotionSystem
{
    Logger::Logger()
        : messageQueue(nullptr),
          taskHandle(nullptr),
          mutex(nullptr),
          droppedMessages(0),
          queueWarningThreshold(50),   // 50% of queue size
          queueCriticalThreshold(75),  // 75% of queue size
          isCritical(false)
    {
    }

    Logger::~Logger()
    {
        if (taskHandle != nullptr)
        {
            vTaskDelete(taskHandle);
            taskHandle = nullptr;
        }
        if (messageQueue != nullptr)
        {
            vQueueDelete(messageQueue);
            messageQueue = nullptr;
        }
        if (mutex != nullptr)
        {
            vSemaphoreDelete(mutex);
            mutex = nullptr;
        }
    }

    void Logger::begin()
    {
        // Initialize Serial first
        Serial.begin(System::SERIAL_BAUD_RATE);
        delay(System::STARTUP_DELAY_MS);
        while (!Serial)
        {
            delay(10);
        }

        // Create mutex first
        mutex = xSemaphoreCreateMutex();
        if (mutex == nullptr)
        {
            Serial.println("ERROR: Failed to create logger mutex");
            while (1)
            {
                delay(1000);
            }
        }

        // Create message queue
        messageQueue = xQueueCreate(LOGGER_QUEUE_SIZE, sizeof(LogMessage));
        if (messageQueue == nullptr)
        {
            Serial.println("ERROR: Failed to create logger message queue");
            while (1)
            {
                delay(1000);
            }
        }

        // Create logger task
        BaseType_t taskCreated =
            xTaskCreatePinnedToCore(loggerTask, "LoggerTask", TASK_STACK_SIZE, this, TASK_PRIORITY, &taskHandle,
                                    0);  // Run on core 0

        if (taskCreated != pdPASS)
        {
            Serial.println("ERROR: Failed to create logger task");
            while (1)
            {
                delay(1000);
            }
        }

        // Wait for task to start
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    bool Logger::isQueueFull() const
    {
        return uxQueueSpacesAvailable(messageQueue) == 0;
    }

    uint32_t Logger::getQueueSize() const
    {
        return uxQueueMessagesWaiting(messageQueue);
    }

    uint32_t Logger::getQueueSpaces() const
    {
        return uxQueueSpacesAvailable(messageQueue);
    }

    uint32_t Logger::getDroppedMessages() const
    {
        return droppedMessages;
    }

    void Logger::setQueueWarningThreshold(uint32_t threshold)
    {
        queueWarningThreshold = threshold;
    }

    void Logger::setQueueCriticalThreshold(uint32_t threshold)
    {
        queueCriticalThreshold = threshold;
    }

    void Logger::clearQueue()
    {
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
        {
            xQueueReset(messageQueue);
            isCritical = false;
            xSemaphoreGive(mutex);
        }
    }

    void Logger::checkQueueHealth()
    {
        uint32_t currentSize   = getQueueSize();
        uint32_t queueCapacity = LOGGER_QUEUE_SIZE;
        uint32_t usagePercent  = (currentSize * 100) / queueCapacity;

        if (usagePercent >= queueCriticalThreshold && !isCritical)
        {
            isCritical = true;
            handleCriticalState();
        }
        else if (usagePercent < queueCriticalThreshold)
        {
            isCritical = false;
        }
    }

    void Logger::handleCriticalState()
    {
        char warning[128];
        snprintf(warning, sizeof(warning),
                 "CRITICAL: Logger queue at %" PRIu32 "%% capacity (%" PRIu32 "/%" PRIu32 " messages)",
                 (getQueueSize() * 100) / LOGGER_QUEUE_SIZE, getQueueSize(), LOGGER_QUEUE_SIZE);

        // Try to send critical warning
        if (xQueueSend(messageQueue, &warning, pdMS_TO_TICKS(QUEUE_WAIT_MS)) != pdPASS)
        {
            // If we can't send the warning, clear the queue and try again
            clearQueue();
            xQueueSend(messageQueue, &warning, 0);
        }
    }

    bool Logger::sendMessage(const char* message, bool isFlashString, bool addNewline)
    {
        if (messageQueue == nullptr)
            return false;

        checkQueueHealth();

        LogMessage logMsg;
        if (isFlashString)
        {
            strncpy_P(logMsg.message, message, MAX_MESSAGE_LENGTH - 1);
        }
        else
        {
            strncpy(logMsg.message, message, MAX_MESSAGE_LENGTH - 1);
        }
        logMsg.message[MAX_MESSAGE_LENGTH - 1] = '\0';
        logMsg.isFlashString                   = isFlashString;
        logMsg.addNewline                      = addNewline;

        // Try to send immediately
        if (xQueueSend(messageQueue, &logMsg, 0) == pdPASS)
        {
            return true;
        }

        // If queue is full, wait for a short time
        if (xQueueSend(messageQueue, &logMsg, pdMS_TO_TICKS(QUEUE_WAIT_MS)) == pdPASS)
        {
            return true;
        }

        // If still can't send, increment dropped messages counter
        droppedMessages++;

        // Log warning only every 100 dropped messages to avoid flooding
        if (droppedMessages % 100 == 0)
        {
            char warning[64];
            snprintf(warning, sizeof(warning),
                     "WARNING: %" PRIu32 " messages dropped (Queue full, %" PRIu32 "%% capacity)", droppedMessages,
                     (getQueueSize() * 100) / LOGGER_QUEUE_SIZE);

            if (xQueueSend(messageQueue, &warning, pdMS_TO_TICKS(QUEUE_WAIT_MS)) != pdPASS)
            {
                // If we can't even log the warning, clear the queue and try again
                clearQueue();
                xQueueSend(messageQueue, &warning, 0);
            }
        }

        return false;
    }

    bool Logger::sendFormattedMessage(const char* format, bool isFlashString, bool addNewline, va_list args)
    {
        if (messageQueue == nullptr)
            return false;

        LogMessage logMsg;
        if (isFlashString)
        {
            vsnprintf_P(logMsg.message, MAX_MESSAGE_LENGTH, format, args);
        }
        else
        {
            vsnprintf(logMsg.message, MAX_MESSAGE_LENGTH, format, args);
        }
        logMsg.isFlashString = isFlashString;
        logMsg.addNewline    = addNewline;

        // Try to send immediately
        if (xQueueSend(messageQueue, &logMsg, 0) == pdPASS)
        {
            return true;
        }

        // If queue is full, wait for a short time
        if (xQueueSend(messageQueue, &logMsg, pdMS_TO_TICKS(QUEUE_WAIT_MS)) == pdPASS)
        {
            return true;
        }

        // If still can't send, increment dropped messages counter
        droppedMessages++;
        return false;
    }

    void Logger::sendHexValue(uint64_t value, uint8_t digits, bool addNewline)
    {
        if (messageQueue == nullptr)
            return;

        LogMessage logMsg;
        char       format[16];
        snprintf(format, sizeof(format), "0x%%0%u" PRIX64, digits);
        snprintf(logMsg.message, MAX_MESSAGE_LENGTH, format, value);
        logMsg.isFlashString = false;
        logMsg.addNewline    = addNewline;

        if (xQueueSend(messageQueue, &logMsg, 0) != pdTRUE)
        {
            // Queue is full, drop the message
            Serial.println(F("Logger queue is full, message dropped"));
        }
    }

    void Logger::sendDecAsHex(uint64_t value, uint8_t digits, bool addNewline)
    {
        if (messageQueue == nullptr)
            return;

        LogMessage logMsg;
        char       temp[32];
        snprintf(temp, sizeof(temp), "%llX", value);
        snprintf(logMsg.message, MAX_MESSAGE_LENGTH, "0x%*s", digits, temp);
        logMsg.isFlashString = false;
        logMsg.addNewline    = addNewline;

        if (xQueueSend(messageQueue, &logMsg, 0) != pdTRUE)
        {
            // Queue is full, drop the message
            Serial.println(F("Logger queue is full, message dropped"));
        }
    }

    // Regular logging methods
    void Logger::log(const char* message)
    {
        sendMessage(message, false, false);
    }

    void Logger::log(const String& message)
    {
        sendMessage(message.c_str(), false, false);
    }

    void Logger::log(const __FlashStringHelper* message)
    {
        sendMessage((const char*)message, true, false);
    }

    void Logger::logf(const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        sendFormattedMessage(format, false, false, args);
        va_end(args);
    }

    void Logger::logf(const __FlashStringHelper* format, ...)
    {
        va_list args;
        va_start(args, format);
        sendFormattedMessage((const char*)format, true, false, args);
        va_end(args);
    }

    // Newline logging methods
    void Logger::logln(const char* message)
    {
        sendMessage(message, false, true);
    }

    void Logger::logln(const String& message)
    {
        sendMessage(message.c_str(), false, true);
    }

    void Logger::logln(const __FlashStringHelper* message)
    {
        sendMessage((const char*)message, true, true);
    }

    void Logger::loglnf(const char* format, ...)
    {
        va_list args;
        va_start(args, format);
        sendFormattedMessage(format, false, true, args);
        va_end(args);
    }

    void Logger::loglnf(const __FlashStringHelper* format, ...)
    {
        va_list args;
        va_start(args, format);
        sendFormattedMessage((const char*)format, true, true, args);
        va_end(args);
    }

    // HEX value logging methods
    void Logger::logHex(uint8_t value, uint8_t digits)
    {
        sendHexValue(value, digits, false);
    }

    void Logger::logHex(uint16_t value, uint8_t digits)
    {
        sendHexValue(value, digits, false);
    }

    void Logger::logHex(uint32_t value, uint8_t digits)
    {
        sendHexValue(value, digits, false);
    }

    void Logger::logHex(uint64_t value, uint8_t digits)
    {
        sendHexValue(value, digits, false);
    }

    void Logger::logHex(int8_t value, uint8_t digits)
    {
        sendHexValue(static_cast<uint8_t>(value), digits, false);
    }

    void Logger::logHex(int16_t value, uint8_t digits)
    {
        sendHexValue(static_cast<uint16_t>(value), digits, false);
    }

    void Logger::logHex(int32_t value, uint8_t digits)
    {
        sendHexValue(static_cast<uint32_t>(value), digits, false);
    }

    void Logger::logHex(int64_t value, uint8_t digits)
    {
        sendHexValue(static_cast<uint64_t>(value), digits, false);
    }

    // HEX value logging with newline
    void Logger::loglnHex(uint8_t value, uint8_t digits)
    {
        sendHexValue(value, digits, true);
    }

    void Logger::loglnHex(uint16_t value, uint8_t digits)
    {
        sendHexValue(value, digits, true);
    }

    void Logger::loglnHex(uint32_t value, uint8_t digits)
    {
        sendHexValue(value, digits, true);
    }

    void Logger::loglnHex(uint64_t value, uint8_t digits)
    {
        sendHexValue(value, digits, true);
    }

    void Logger::loglnHex(int8_t value, uint8_t digits)
    {
        sendHexValue(static_cast<uint8_t>(value), digits, true);
    }

    void Logger::loglnHex(int16_t value, uint8_t digits)
    {
        sendHexValue(static_cast<uint16_t>(value), digits, true);
    }

    void Logger::loglnHex(int32_t value, uint8_t digits)
    {
        sendHexValue(static_cast<uint32_t>(value), digits, true);
    }

    void Logger::loglnHex(int64_t value, uint8_t digits)
    {
        sendHexValue(static_cast<uint64_t>(value), digits, true);
    }

    // Decimal to HEX logging methods
    void Logger::logDecAsHex(uint8_t value, uint8_t digits)
    {
        sendDecAsHex(value, digits, false);
    }

    void Logger::logDecAsHex(uint16_t value, uint8_t digits)
    {
        sendDecAsHex(value, digits, false);
    }

    void Logger::logDecAsHex(uint32_t value, uint8_t digits)
    {
        sendDecAsHex(value, digits, false);
    }

    void Logger::logDecAsHex(uint64_t value, uint8_t digits)
    {
        sendDecAsHex(value, digits, false);
    }

    void Logger::logDecAsHex(int8_t value, uint8_t digits)
    {
        sendDecAsHex(static_cast<uint8_t>(value), digits, false);
    }

    void Logger::logDecAsHex(int16_t value, uint8_t digits)
    {
        sendDecAsHex(static_cast<uint16_t>(value), digits, false);
    }

    void Logger::logDecAsHex(int32_t value, uint8_t digits)
    {
        sendDecAsHex(static_cast<uint32_t>(value), digits, false);
    }

    void Logger::logDecAsHex(int64_t value, uint8_t digits)
    {
        sendDecAsHex(static_cast<uint64_t>(value), digits, false);
    }

    // Decimal to HEX logging with newline
    void Logger::loglnDecAsHex(uint8_t value, uint8_t digits)
    {
        sendDecAsHex(value, digits, true);
    }

    void Logger::loglnDecAsHex(uint16_t value, uint8_t digits)
    {
        sendDecAsHex(value, digits, true);
    }

    void Logger::loglnDecAsHex(uint32_t value, uint8_t digits)
    {
        sendDecAsHex(value, digits, true);
    }

    void Logger::loglnDecAsHex(uint64_t value, uint8_t digits)
    {
        sendDecAsHex(value, digits, true);
    }

    void Logger::loglnDecAsHex(int8_t value, uint8_t digits)
    {
        sendDecAsHex(static_cast<uint8_t>(value), digits, true);
    }

    void Logger::loglnDecAsHex(int16_t value, uint8_t digits)
    {
        sendDecAsHex(static_cast<uint16_t>(value), digits, true);
    }

    void Logger::loglnDecAsHex(int32_t value, uint8_t digits)
    {
        sendDecAsHex(static_cast<uint32_t>(value), digits, true);
    }

    void Logger::loglnDecAsHex(int64_t value, uint8_t digits)
    {
        sendDecAsHex(static_cast<uint64_t>(value), digits, true);
    }

    void Logger::processMessage(const LogMessage& msg)
    {
        if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
        {
            if (msg.isFlashString)
            {
                if (msg.addNewline)
                {
                    Serial.println(F(msg.message));
                }
                else
                {
                    Serial.print(F(msg.message));
                }
            }
            else
            {
                if (msg.addNewline)
                {
                    Serial.println(msg.message);
                }
                else
                {
                    Serial.print(msg.message);
                }
            }
            xSemaphoreGive(mutex);
        }
    }

    void Logger::loggerTask(void* parameter)
    {
        Logger*    logger = static_cast<Logger*>(parameter);
        LogMessage msg;

        while (true)
        {
            if (xQueueReceive(logger->messageQueue, &msg, portMAX_DELAY) == pdPASS)
            {
                logger->processMessage(msg);
            }
            vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
        }
    }
}  // namespace MotionSystem