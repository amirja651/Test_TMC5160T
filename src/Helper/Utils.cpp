#include "Helper/Utils.h"
#include "Helper/System.h"

namespace MotionSystem
{
    Utils* Utils::instance = nullptr;

    Utils& Utils::getInstance()
    {
        if (!instance)
        {
            instance = new Utils();
        }

        return *instance;
    }

    Utils::Utils() {}

    Types::MicronPosition Utils::countsToMicrons(Types::EncoderPosition counts)
    {
        return static_cast<Types::MicronPosition>(counts) / System::ENCODER_COUNTS_PER_MICRON;
    }

    Types::EncoderPosition Utils::micronsToEncCounts(Types::MicronPosition microns)
    {
        return static_cast<Types::EncoderPosition>(microns * System::ENCODER_COUNTS_PER_MICRON);
    }

    Types::PixelPosition Utils::countsToPixels(Types::EncoderPosition counts)
    {
        return countsToMicrons(counts) / System::PIXEL_SIZE;
    }

    bool Utils::isNumber(const String& str)
    {
        for (int i = 0; i < str.length(); i++)
        {
            if (!isDigit(str.charAt(i)))
            {
                return false;
            }
        }
        return true;
    }
}  // namespace MotionSystem