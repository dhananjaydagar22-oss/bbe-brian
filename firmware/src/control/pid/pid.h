#pragma once

#include <Arduino.h>

namespace BeeBrain {

class PIDController {
public:
    PIDController(float kp = 0.0f,
                  float ki = 0.0f,
                  float kd = 0.0f,
                  float outMin = -1.0f,
                  float outMax = 1.0f);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float outMin, float outMax);
    void reset();

    /**
     * @brief Compute PID output.
     * @param setpoint Desired target value.
     * @param measurement Current measured value.
     * @param dt Elapsed time in seconds.
     * @return Clamped control output.
     */
    float compute(float setpoint, float measurement, float dt);

private:
    float _kp;
    float _ki;
    float _kd;
    float _outMin;
    float _outMax;

    float _integral     = 0.0f;
    float _prevError    = 0.0f;
    float _prevDerivative = 0.0f;
    bool  _firstRun     = true;
};

} // namespace BeeBrain
