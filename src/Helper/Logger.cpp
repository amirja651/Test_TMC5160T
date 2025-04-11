#include "Helper/Logger.h"

namespace MotionSystem
{
    Logger::Logger() : messageQueue(nullptr), loggerTaskHandle(nullptr), mutex(nullptr) {}

    Logger::~Logger()
    {
        if (loggerTaskHandle != nullptr)
        {
            vTaskDelete(loggerTaskHandle);
        }
        if (messageQueue != nullptr)
        {
            vQueueDelete(messageQueue);
        }
        if (mutex != nullptr)
        {
            vSemaphoreDelete(mutex);
        }
    }

    void Logger::begin()
    {
        // Create message queue
        messageQueue = xQueueCreate(BUFFER_SIZE, sizeof(LogMessage));
        if (messageQueue == nullptr)
        {
            Serial.println(F("Failed to create logger message queue"));
            return;
        }

        // Create mutex
        mutex = xSemaphoreCreateMutex();
        if (mutex == nullptr)
        {
            Serial.println(F("Failed to create logger mutex"));
            return;
        }

        // Create logger task
        xTaskCreate(loggerTask, "LoggerTask", TASK_STACK_SIZE, this, TASK_PRIORITY, &loggerTaskHandle);
        if (loggerTaskHandle == nullptr)
        {
            Serial.println(F("Failed to create logger task"));
            return;
        }
    }

    void Logger::sendMessage(const char* message, bool isFlashString, bool addNewline)
    {
        if (messageQueue == nullptr)
            return;

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

        if (xQueueSend(messageQueue, &logMsg, 0) != pdTRUE)
        {
            // Queue is full, drop the message
            Serial.println(F("Logger queue is full, message dropped"));
        }
    }

    void Logger::sendFormattedMessage(const char* format, bool isFlashString, bool addNewline, va_list args)
    {
        if (messageQueue == nullptr)
            return;

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

        if (xQueueSend(messageQueue, &logMsg, 0) != pdTRUE)
        {
            // Queue is full, drop the message
            Serial.println(F("Logger queue is full, message dropped"));
        }
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

    void Logger::loggerTask(void* parameter)
    {
        Logger*    logger = static_cast<Logger*>(parameter);
        LogMessage msg;

        while (1)
        {
            if (xQueueReceive(logger->messageQueue, &msg, portMAX_DELAY) == pdTRUE)
            {
                if (xSemaphoreTake(logger->mutex, portMAX_DELAY) == pdTRUE)
                {
                    logger->processMessage(msg);
                    xSemaphoreGive(logger->mutex);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
        }
    }

    void Logger::processMessage(const LogMessage& msg)
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
    }
}  // namespace MotionSystem