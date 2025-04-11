#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>
#include "Helper/Types.h"

namespace MotionSystem
{
    class Utils
    {
    public:
        static Utils&          getInstance();
        Types::MicronPosition  countsToMicrons(Types::EncoderPosition counts);
        Types::EncoderPosition micronsToEncCounts(Types::MicronPosition microns);
        Types::PixelPosition   countsToPixels(Types::EncoderPosition counts);
        bool                   isNumber(const String& str);

    private:
        Utils();
        Utils(const Utils&)                   = delete;
        Utils&        operator=(const Utils&) = delete;
        static Utils* instance;
    };
}  // namespace MotionSystem

#endif