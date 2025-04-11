#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "Config.h"
#include "Encoders/EncoderInterface.h"
#include "Helper/Logger.h"
#include "esp_timer.h"

namespace MotionSystem
{
    using namespace MotionSystem::Types;

    struct PIDStats
    {
        uint32_t updateCount;
        float    averageUpdateTime;
        uint32_t errorCount;
        float    maxError;
        float    currentError;
        float    currentOutput;
    };

    class PIDController
    {
    public:
        PIDController(EncoderInterface* encoder);
        ~PIDController();

        // Core functionality
        void            init();
        void            setTargetPosition(Types::EncoderPosition targetPosition);
        EncoderPosition getTargetPosition() const;
        EncoderPosition update();
        static void     pidTask(void* parameter);
        void            startTask();
        void            stopTask();

        // Configuration and validation
        bool validateConfig() const;
        void setGains(float kp, float ki, float kd);
        void setIntegralLimit(float limit);
        void setOutputLimit(float limit);

        // Status and monitoring
        void  getStats(PIDStats& stats) const;
        bool  isRunning() const;
        float getCurrentError() const;
        float getCurrentOutput() const;

    private:
        EncoderInterface* encoder;
        EncoderPosition   targetPosition;
        EncoderPosition   lastEncoderPosition;
        float             integral;
        float             lastError;
        EncoderPosition   output;
        Timestamp         lastPidTime;
        TaskHandle_t      taskHandle;

        // Configuration parameters
        float kp;
        float ki;
        float kd;
        float maxIntegral;
        float maxOutput;

        // Performance monitoring
        bool     shouldStop;
        uint32_t updateCount;
        float    totalUpdateTime;
        uint32_t errorCount;
        float    maxError;
        float    currentError;
        float    currentOutput;

        static constexpr EncoderPosition INVALID_POSITION = -1;
        static constexpr float           MIN_UPDATE_TIME  = 0.0001f;  // 100 microseconds
    };
}  // namespace MotionSystem

#endif  // PID_CONTROLLER_H