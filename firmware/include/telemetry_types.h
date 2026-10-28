#pragma once

#include <Arduino.h>

/**
 * @file telemetry_types.h
 * @brief Plain-old-data (POD) types for sensor telemetry, estimation, and health flags.
 *
 * Designed to strictly map to the Bee-Brain Telemetry Specification and Pydantic
 * TelemetryPacket model in dashboard/services/telemetry.py.
 */

namespace BeeBrain {

// =============================================================================
// Sensor Status Flags
// =============================================================================
struct SensorStatusData {
    bool tof          = false; // VL53L0X healthy and communicating
    bool optical_flow = false; // ADNS3080 healthy and tracking
    bool imu          = false; // MPU6050 healthy and calibrated
    bool barometer    = false; // Optional future barometer
};

// =============================================================================
// Subsystem Data Blocks
// =============================================================================
struct ToFData {
    float distance_mm       = 0.0f;
    float signal_rate       = 0.0f;
    bool  measurement_valid = false;
};

struct OpticalFlowData {
    float delta_x         = 0.0f; // Raw counts
    float delta_y         = 0.0f; // Raw counts
    float velocity_x_cm_s = 0.0f; // Scaled metric velocity
    float velocity_y_cm_s = 0.0f; // Scaled metric velocity
    float quality         = 0.0f; // Normalized 0.0 to 1.0 (from SQUAL)
    bool  tracking_ok     = false;
};

struct IMUData {
    // Attitude angles (degrees)
    float roll_deg      = 0.0f;
    float pitch_deg     = 0.0f;
    float yaw_deg       = 0.0f;

    // Angular velocities (deg/s)
    float gyro_x_dps    = 0.0f;
    float gyro_y_dps    = 0.0f;
    float gyro_z_dps    = 0.0f;

    // Linear accelerations (m/s^2)
    float accel_x_m_s2  = 0.0f;
    float accel_y_m_s2  = 0.0f;
    float accel_z_m_s2  = 9.80665f;
};

struct FlightData {
    float target_altitude_cm     = 0.0f;
    float altitude_error_cm      = 0.0f;
    float vertical_velocity_cm_s = 0.0f;
    float roll_deg               = 0.0f;
    float pitch_deg              = 0.0f;
    float yaw_deg                = 0.0f;
    bool  armed                  = false;
    char  flight_mode[16]        = "BENCH_TEST";
};

struct BatteryData {
    float voltage_v           = 0.0f;
    float current_a           = 0.0f;
    float power_w             = 0.0f;
    float state_of_charge_pct = 0.0f;
    float temperature_c       = 25.0f;
};

struct MotorData {
    uint32_t pwm[4] = {0, 0, 0, 0}; // 0 to 65535 duty cycles
};

struct CommunicationData {
    bool     link_ok          = true;
    float    packet_rate_hz   = 50.0f;
    float    latency_ms       = 0.0f;
    float    rssi_dbm         = 0.0f;
    uint32_t packets_received = 0;
    uint32_t packets_dropped  = 0;
    float    packet_loss_pct  = 0.0f;
};

struct SystemData {
    float    uptime_s           = 0.0f;
    float    loop_rate_hz       = 100.0f;
    float    processing_time_ms = 0.0f;
    uint32_t error_count        = 0;
    uint32_t warning_count      = 0;
};

// =============================================================================
// Unified Telemetry Snapshot (Thread-safe shared model)
// =============================================================================
struct FullTelemetrySnapshot {
    // Legacy top-level contract fields (strictly preserved for backwards compatibility)
    float altitude_cm  = 0.0f;
    float drift_x      = 0.0f;
    float drift_y      = 0.0f;
    float battery_v    = 0.0f;
    uint32_t motor_pwm[4] = {0, 0, 0, 0};
    bool  link_ok      = true;
    SensorStatusData sensor_status;

    // Extended contract fields
    float    timestamp_s = 0.0f;
    uint32_t sequence    = 0;

    ToFData           tof;
    OpticalFlowData   optical_flow;
    IMUData           imu;
    FlightData        flight;
    BatteryData       battery;
    MotorData         motors;
    CommunicationData communications;
    SystemData        system;
};

} // namespace BeeBrain
