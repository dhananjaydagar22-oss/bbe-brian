#include "pid.h"
#include <algorithm>

namespace BeeBrain {

PIDController::PIDController(float kp, float ki, float kd, float outMin, float outMax)
    : _kp(kp), _ki(ki), _kd(kd), _outMin(outMin), _outMax(outMax) {
    reset();
}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setOutputLimits(float outMin, float outMax) {
    _outMin = outMin;
    _outMax = outMax;
}

void PIDController::reset() {
    _integral       = 0.0f;
    _prevError      = 0.0f;
    _prevDerivative = 0.0f;
    _firstRun       = true;
}

float PIDController::compute(float setpoint, float measurement, float dt) {
    if (dt <= 0.0001f) {
        return 0.0f;
    }

    float error = setpoint - measurement;

    // Proportional term
    float pTerm = _kp * error;

    // Integral term with anti-windup clamping
    _integral += _ki * error * dt;
    _integral = std::max(_outMin, std::min(_outMax, _integral));

    // Derivative term with low-pass filtering
    float derivative = 0.0f;
    if (!_firstRun) {
        float rawDerivative = (error - _prevError) / dt;
        // Low-pass filter (cutoff ~20 Hz)
        constexpr float filterAlpha = 0.7f;
        derivative = (filterAlpha * _prevDerivative) + ((1.0f - filterAlpha) * rawDerivative);
        _prevDerivative = derivative;
    } else {
        _firstRun = false;
    }
    _prevError = error;

    float dTerm = _kd * derivative;

    // Total output clamped to limits
    float output = pTerm + _integral + dTerm;
    return std::max(_outMin, std::min(_outMax, output));
}

} // namespace BeeBrain
