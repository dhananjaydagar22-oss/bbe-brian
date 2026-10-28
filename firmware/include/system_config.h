#pragma once

#include <Arduino.h>

/**
 * @file system_config.h
 * @brief System parameters, task timing, filter gains, and network constants.
 */

namespace BeeBrain {
namespace Config {

    // =========================================================================
    // Real-Time Task Execution Frequencies & Delays
    // =========================================================================
    constexpr uint32_t SENSOR_TASK_FREQ_HZ     = 100; // Fast sensor & estimator loop
    constexpr uint32_t SENSOR_TASK_PERIOD_MS   = 1000 / SENSOR_TASK_FREQ_HZ; // 10 ms
    constexpr float    SENSOR_TASK_DT_S        = 1.0f / static_cast<float>(SENSOR_TASK_FREQ_HZ);

    constexpr uint32_t TELEMETRY_TASK_FREQ_HZ  = 50;  // Telemetry stream loop (20-50 Hz)
    constexpr uint32_t TELEMETRY_TASK_PERIOD_MS= 1000 / TELEMETRY_TASK_FREQ_HZ; // 20 ms
    constexpr float    TELEMETRY_TASK_DT_S     = 1.0f / static_cast<float>(TELEMETRY_TASK_FREQ_HZ);

    // =========================================================================
    // FreeRTOS Task Allocations & Priorities
    // =========================================================================
    constexpr BaseType_t CORE_REALTIME_SENSORS = 1;   // Core 1: Deterministic sensor loop
    constexpr BaseType_t CORE_COMMUNICATIONS   = 0;   // Core 0: Wi-Fi, UDP, JSON formatting

    constexpr UBaseType_t PRIO_SENSOR_TASK     = 5;   // Higher priority
    constexpr UBaseType_t PRIO_TELEMETRY_TASK  = 2;   // Normal priority

    constexpr uint32_t STACK_SENSOR_TASK       = 8192; // 8 KB stack
    constexpr uint32_t STACK_TELEMETRY_TASK    = 8192; // 8 KB stack

    // =========================================================================
    // Network & Telemetry Configuration
    // =========================================================================
    constexpr uint16_t UDP_TELEMETRY_PORT      = 8888;
    constexpr uint32_t SERIAL_BAUD_RATE        = 115200;

    // Default Wi-Fi Access Point / Network Credentials
    // (Can be overridden by local credentials or AP mode)
    constexpr const char* DEFAULT_WIFI_SSID    = "Drobee-Relay";
    constexpr const char* DEFAULT_WIFI_PASS    = "drobee123";
    constexpr const char* PI_FORWARDER_IP      = "192.168.4.2"; // Raspberry Pi 3 B+ IP

    // =========================================================================
    // Sensor Calibration & Physical Constraints
    // =========================================================================
    // MPU6050
    constexpr uint16_t MPU6050_CALIB_SAMPLES   = 200; // Samples collected at startup
    constexpr float    GRAVITY_MSS             = 9.80665f;
    constexpr float    ATTITUDE_FILTER_ALPHA   = 0.98f; // Complementary filter weight (gyro)

    // VL53L0X
    constexpr float    TOF_MIN_VALID_RANGE_MM  = 30.0f;
    constexpr float    TOF_MAX_VALID_RANGE_MM  = 2000.0f;
    constexpr float    TOF_SIGMA_LIMIT_MM      = 60.0f;

    // ADNS3080 Optical Flow
    constexpr uint8_t  FLOW_MIN_SQUAL          = 15;   // Minimum surface quality (0-255)
    constexpr float    FLOW_FOV_RAD            = 0.21f;// ~12 degrees optical field of view
    constexpr float    FLOW_PIXELS_TOTAL       = 30.0f;// 30x30 optical sensor resolution

    // Battery Thresholds (1S LiPo)
    constexpr float    BATT_VOLTAGE_MIN_V      = 3.30f;// Critical cutoff
    constexpr float    BATT_VOLTAGE_WARN_V     = 3.50f;// Low battery warning
    constexpr float    BATT_VOLTAGE_MAX_V      = 4.20f;// Fully charged

    // Bio-Inspired Parameters
    constexpr float    REICHARDT_TAU_S         = 0.05f;// Time constant for delay filter (50 ms)
    constexpr float    LGMD_COLLISION_VEL_THRESH = -150.0f; // cm/s downward closing velocity

    // Bench Actuator Parameters (SG90 Servo)
    constexpr uint32_t SERVO_PWM_FREQ_HZ       = 50;   // 50 Hz standard analog servo
    constexpr uint8_t  SERVO_PWM_RESOLUTION_BITS = 14; // 14-bit resolution
    constexpr uint32_t SERVO_MIN_PULSE_US      = 1000; // 1000 µs (0 degrees)
    constexpr uint32_t SERVO_MAX_PULSE_US      = 2000; // 2000 µs (180 degrees)

} // namespace Config
} // namespace BeeBrain
