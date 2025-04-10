#include "Utils.h"

MotionSystem::Utils* MotionSystem::Utils::instance = nullptr;

MotionSystem::Utils& MotionSystem::Utils::getInstance()
{
    if (!instance)
    {
        instance = new Utils();
    }

    return *instance;
}

MotionSystem::Utils::Utils() {}

MotionSystem::Types::MicronPosition MotionSystem::Utils::countsToMicrons(Types::EncoderPosition counts)
{
    return static_cast<Types::MicronPosition>(counts) / Config::System::ENCODER_COUNTS_PER_MICRON;
}

MotionSystem::Types::EncoderPosition MotionSystem::Utils::micronsToEncCounts(Types::MicronPosition microns)
{
    return static_cast<Types::EncoderPosition>(microns * Config::System::ENCODER_COUNTS_PER_MICRON);
}

MotionSystem::Types::PixelPosition MotionSystem::Utils::countsToPixels(Types::EncoderPosition counts)
{
    return countsToMicrons(counts) / Config::System::PIXEL_SIZE;
}