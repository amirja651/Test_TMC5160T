#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

namespace MotionSystem
{
    class Logger
    {
    public:
        static Logger& getInstance()
        {
            static Logger instance;
            return instance;
        }

        // Delete copy constructor and assignment operator
        Logger(const Logger&)            = delete;
        Logger& operator=(const Logger&) = delete;

        void begin();
        void log(const char* message);
        void log(const String& message);
        void log(const __FlashStringHelper* message);
        void logf(const char* format, ...);
        void logf(const __FlashStringHelper* format, ...);

        // New logln methods
        void logln(const char* message);
        void logln(const String& message);
        void logln(const __FlashStringHelper* message);
        void loglnf(const char* format, ...);
        void loglnf(const __FlashStringHelper* format, ...);

        // HEX value logging methods
        void logHex(uint8_t value, uint8_t digits = 2);
        void logHex(uint16_t value, uint8_t digits = 4);
        void logHex(uint32_t value, uint8_t digits = 8);
        void logHex(uint64_t value, uint8_t digits = 16);
        void logHex(int8_t value, uint8_t digits = 2);
        void logHex(int16_t value, uint8_t digits = 4);
        void logHex(int32_t value, uint8_t digits = 8);
        void logHex(int64_t value, uint8_t digits = 16);

        // HEX value logging with newline
        void loglnHex(uint8_t value, uint8_t digits = 2);
        void loglnHex(uint16_t value, uint8_t digits = 4);
        void loglnHex(uint32_t value, uint8_t digits = 8);
        void loglnHex(uint64_t value, uint8_t digits = 16);
        void loglnHex(int8_t value, uint8_t digits = 2);
        void loglnHex(int16_t value, uint8_t digits = 4);
        void loglnHex(int32_t value, uint8_t digits = 8);
        void loglnHex(int64_t value, uint8_t digits = 16);

        // Decimal to HEX logging methods
        void logDecAsHex(uint8_t value, uint8_t digits = 2);
        void logDecAsHex(uint16_t value, uint8_t digits = 4);
        void logDecAsHex(uint32_t value, uint8_t digits = 8);
        void logDecAsHex(uint64_t value, uint8_t digits = 16);
        void logDecAsHex(int8_t value, uint8_t digits = 2);
        void logDecAsHex(int16_t value, uint8_t digits = 4);
        void logDecAsHex(int32_t value, uint8_t digits = 8);
        void logDecAsHex(int64_t value, uint8_t digits = 16);

        // Decimal to HEX logging with newline
        void loglnDecAsHex(uint8_t value, uint8_t digits = 2);
        void loglnDecAsHex(uint16_t value, uint8_t digits = 4);
        void loglnDecAsHex(uint32_t value, uint8_t digits = 8);
        void loglnDecAsHex(uint64_t value, uint8_t digits = 16);
        void loglnDecAsHex(int8_t value, uint8_t digits = 2);
        void loglnDecAsHex(int16_t value, uint8_t digits = 4);
        void loglnDecAsHex(int32_t value, uint8_t digits = 8);
        void loglnDecAsHex(int64_t value, uint8_t digits = 16);

    private:
        Logger();  // Private constructor
        ~Logger();

        static constexpr size_t   BUFFER_SIZE        = 1024;  // Size of the circular buffer
        static constexpr size_t   MAX_MESSAGE_LENGTH = 128;   // Maximum length of a single message
        static constexpr uint32_t TASK_STACK_SIZE    = 4096;
        static constexpr uint32_t TASK_PRIORITY      = 1;
        static constexpr uint32_t TASK_DELAY_MS      = 10;

        struct LogMessage
        {
            char message[MAX_MESSAGE_LENGTH];
            bool isFlashString;
            bool addNewline;
        };

        QueueHandle_t     messageQueue;
        TaskHandle_t      loggerTaskHandle;
        SemaphoreHandle_t mutex;

        static void loggerTask(void* parameter);
        void        processMessage(const LogMessage& msg);
        void        sendMessage(const char* message, bool isFlashString, bool addNewline);
        void        sendFormattedMessage(const char* format, bool isFlashString, bool addNewline, va_list args);
        void        sendHexValue(uint64_t value, uint8_t digits, bool addNewline);
        void        sendDecAsHex(uint64_t value, uint8_t digits, bool addNewline);
    };
}  // namespace MotionSystem

#endif  // LOGGER_H