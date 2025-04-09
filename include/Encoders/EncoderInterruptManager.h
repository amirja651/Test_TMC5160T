#pragma once

#include <Arduino.h>
#include <mutex>
#include <vector>

namespace MotionSystem
{
    class EncoderInterruptManager
    {
    public:
        static EncoderInterruptManager& getInstance()
        {
            static EncoderInterruptManager instance;
            return instance;
        }

        bool allocateInterrupt(uint8_t pin, void (*isr)(void*), void* arg)
        {
            std::lock_guard<std::mutex> lock(mutex);

            if (!isPinAvailable(pin))
            {
                return false;
            }

            if (pin > 39)  // ESP32 pin range
            {
                return false;
            }

            attachInterruptArg(pin, isr, arg, RISING);
            usedPins.push_back(pin);
            return true;
        }

        void freeInterrupt(uint8_t pin)
        {
            std::lock_guard<std::mutex> lock(mutex);

            detachInterrupt(pin);
            auto it = std::find(usedPins.begin(), usedPins.end(), pin);
            if (it != usedPins.end())
            {
                usedPins.erase(it);
            }
        }

        bool isPinAvailable(uint8_t pin) const
        {
            return std::find(usedPins.begin(), usedPins.end(), pin) == usedPins.end();
        }

    private:
        EncoderInterruptManager()  = default;
        ~EncoderInterruptManager() = default;

        // Prevent copying
        EncoderInterruptManager(const EncoderInterruptManager&)            = delete;
        EncoderInterruptManager& operator=(const EncoderInterruptManager&) = delete;

        std::vector<uint8_t> usedPins;
        std::mutex           mutex;
    };
}  // namespace MotionSystem