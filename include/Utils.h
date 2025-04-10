#ifndef UTILS_H
#define UTILS_H

#include "Config.h"
#include "Types.h"

namespace MotionSystem
{
    class Utils
    {
    public:
        static Utils&          getInstance();
        Types::MicronPosition  countsToMicrons(Types::EncoderPosition counts);
        Types::EncoderPosition micronsToEncCounts(Types::MicronPosition microns);
        Types::PixelPosition   countsToPixels(Types::EncoderPosition counts);

    private:
        Utils();
        Utils(const Utils&)                   = delete;
        Utils&        operator=(const Utils&) = delete;
        static Utils* instance;
    };
}  // namespace MotionSystem

#endif