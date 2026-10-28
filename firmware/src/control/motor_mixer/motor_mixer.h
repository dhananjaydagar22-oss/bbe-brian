#pragma once

#include <Arduino.h>
#include "include/pin_config.h"
#include "include/system_config.h"
#include "include/telemetry_types.h"

namespace BeeBrain {

class MotorMixer {
public:
    MotorMixer();

    /**
     * @brief Initialize LEDC PWM channels for bench SG90 servo and motor outputs.
     * All motors are initialized to 0 duty cycle (disarmed).
     */
    bool init();

    /**
     * @brief Set SG90 bench test actuator servo position.
     * @param angleDeg Angle in degrees (0.0 to 180.0).
     */
    void setServoAngle(float angleDeg);

    /**
     * @brief Update bench motor outputs.
     * NOTE: Strictly suppressed if not armed in bench test mode.
     * @param motors MotorData containing commanded PWM values [0-65535].
     * @param armed True only when bench test motor output is explicitly authorized.
     */
    void updateMotors(const MotorData& motors, bool armed);

    /**
     * @brief Emergency disarm: immediately cuts all PWM channels to 0.
     */
    void emergencyDisarm();

    /**
     * @brief Run automatic bench servo sweep step (for pan-tilt / test fixture).
     */
    void stepBenchServoSweep();

private:
    bool  _isInitialized = false;
    float _currentServoAngle = 90.0f;
    float _servoSweepDir     = 1.0f;

    // LEDC Channel Allocations
    static constexpr uint8_t LEDC_CHAN_SERVO = 4;
    static constexpr uint8_t LEDC_CHAN_M1    = 0;
    static constexpr uint8_t LEDC_CHAN_M2    = 1;
    static constexpr uint8_t LEDC_CHAN_M3    = 2;
    static constexpr uint8_t LEDC_CHAN_M4    = 3;
};

} // namespace BeeBrain
