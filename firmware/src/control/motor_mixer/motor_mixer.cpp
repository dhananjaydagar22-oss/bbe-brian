#include "motor_mixer.h"
#include <algorithm>

namespace BeeBrain {

MotorMixer::MotorMixer() {}

bool MotorMixer::init() {
    // 1. Setup SG90 Bench Servo PWM on GPIO 44 via LEDC
    // 50 Hz PWM frequency, 14-bit resolution
    ledcSetup(LEDC_CHAN_SERVO, Config::SERVO_PWM_FREQ_HZ, Config::SERVO_PWM_RESOLUTION_BITS);
    ledcAttachPin(Pins::SERVO_BENCH_PIN, LEDC_CHAN_SERVO);

    // Center servo at 90 degrees
    setServoAngle(90.0f);

    // 2. Setup Bench Motor PWM channels (1 kHz frequency, 16-bit resolution)
    constexpr uint32_t MOTOR_FREQ_HZ = 1000;
    constexpr uint8_t  MOTOR_RES_BITS = 16;

    ledcSetup(LEDC_CHAN_M1, MOTOR_FREQ_HZ, MOTOR_RES_BITS);
    ledcAttachPin(Pins::MOTOR_FL_PIN, LEDC_CHAN_M1);

    ledcSetup(LEDC_CHAN_M2, MOTOR_FREQ_HZ, MOTOR_RES_BITS);
    ledcAttachPin(Pins::MOTOR_FR_PIN, LEDC_CHAN_M2);

    ledcSetup(LEDC_CHAN_M3, MOTOR_FREQ_HZ, MOTOR_RES_BITS);
    ledcAttachPin(Pins::MOTOR_RR_PIN, LEDC_CHAN_M3);

    ledcSetup(LEDC_CHAN_M4, MOTOR_FREQ_HZ, MOTOR_RES_BITS);
    ledcAttachPin(Pins::MOTOR_RL_PIN, LEDC_CHAN_M4);

    // Ensure all motors start strictly at 0 duty cycle
    emergencyDisarm();

    _isInitialized = true;
    return true;
}

void MotorMixer::setServoAngle(float angleDeg) {
    angleDeg = std::max(0.0f, std::min(180.0f, angleDeg));
    _currentServoAngle = angleDeg;

    // Map 0-180 deg to 1000-2000 µs
    float pulseUs = Config::SERVO_MIN_PULSE_US +
                   (angleDeg / 180.0f) * (Config::SERVO_MAX_PULSE_US - Config::SERVO_MIN_PULSE_US);

    // Convert µs to duty cycle ticks at 50 Hz (20,000 µs period) with 14-bit (16384 ticks)
    // Duty = (pulseUs / 20000.0) * 16384
    uint32_t duty = static_cast<uint32_t>((pulseUs / 20000.0f) * 16384.0f);
    ledcWrite(LEDC_CHAN_SERVO, duty);
}

void MotorMixer::stepBenchServoSweep() {
    _currentServoAngle += (_servoSweepDir * 1.5f);
    if (_currentServoAngle >= 150.0f) {
        _currentServoAngle = 150.0f;
        _servoSweepDir = -1.0f;
    } else if (_currentServoAngle <= 30.0f) {
        _currentServoAngle = 30.0f;
        _servoSweepDir = 1.0f;
    }
    setServoAngle(_currentServoAngle);
}

void MotorMixer::updateMotors(const MotorData& motors, bool armed) {
    if (!armed) {
        emergencyDisarm();
        return;
    }

    // When armed in bench mode, write commanded PWM (bounded 0 to 65535)
    uint32_t m1 = std::min(65535UL, static_cast<unsigned long>(motors.pwm[0]));
    uint32_t m2 = std::min(65535UL, static_cast<unsigned long>(motors.pwm[1]));
    uint32_t m3 = std::min(65535UL, static_cast<unsigned long>(motors.pwm[2]));
    uint32_t m4 = std::min(65535UL, static_cast<unsigned long>(motors.pwm[3]));

    ledcWrite(LEDC_CHAN_M1, m1);
    ledcWrite(LEDC_CHAN_M2, m2);
    ledcWrite(LEDC_CHAN_M3, m3);
    ledcWrite(LEDC_CHAN_M4, m4);
}

void MotorMixer::emergencyDisarm() {
    ledcWrite(LEDC_CHAN_M1, 0);
    ledcWrite(LEDC_CHAN_M2, 0);
    ledcWrite(LEDC_CHAN_M3, 0);
    ledcWrite(LEDC_CHAN_M4, 0);
}

} // namespace BeeBrain
