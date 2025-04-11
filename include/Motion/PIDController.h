#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "Encoders/EncoderInterface.h"
#include "Helper/Logger.h"
#include "Helper/Types.h"
#include "esp_timer.h"

namespace MotionSystem
{
    namespace PID
    {
        constexpr float    KP              = 1.2f;   // 0.8f   // Proportional gain
        constexpr float    KI              = 0.15f;  // 0.1f;  // Integral gain
        constexpr float    KD              = 0.08f;  // 0.05f  // Derivative gain
        constexpr uint16_t MAX_INTEGRAL    = 1000;   // Anti-windup limit
        constexpr uint16_t PID_UPDATE_FREQ = 1000;   // PID update frequency in Hz

    }  // namespace PID

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
        PIDController(const char* name, EncoderInterface* encoder);
        ~PIDController();

        // Core functionality
        void                   init();
        void                   setTargetPosition(Types::EncoderPosition targetPosition);
        Types::EncoderPosition getTargetPosition() const;
        Types::EncoderPosition update();
        static void            pidTask(void* parameter);
        void                   startTask();
        void                   stopTask();

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
        EncoderInterface*      encoder;
        const char*            instanceName;
        Types::EncoderPosition targetPosition;
        Types::EncoderPosition lastEncoderPosition;
        float                  integral;
        float                  lastError;
        Types::EncoderPosition output;
        Types::Timestamp       lastPidTime;
        TaskHandle_t           taskHandle;

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

        static constexpr Types::EncoderPosition INVALID_POSITION = -1;
        static constexpr float                  MIN_UPDATE_TIME  = 0.0001f;  // 100 microseconds
    };
}  // namespace MotionSystem

#endif  // PID_CONTROLLER_H