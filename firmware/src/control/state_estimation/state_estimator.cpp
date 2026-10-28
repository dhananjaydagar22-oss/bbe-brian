#include "state_estimator.h"
#include <cmath>

namespace BeeBrain {

StateEstimator::StateEstimator() {
    reset();
}

void StateEstimator::reset() {
    _rollDeg       = 0.0f;
    _pitchDeg      = 0.0f;
    _yawDeg        = 0.0f;
    _altitudeCm    = 0.0f;
    _prevAltitudeCm= 0.0f;
    _verticalVel   = 0.0f;
    _filteredVelX  = 0.0f;
    _filteredVelY  = 0.0f;
}

void StateEstimator::update(IMUData& imu,
                            const ToFData& tof,
                            OpticalFlowData& flow,
                            FlightData& flight,
                            float& altitudeCm,
                            float& driftX,
                            float& driftY,
                            float dt) {
    if (dt <= 0.0001f) {
        dt = Config::SENSOR_TASK_DT_S;
    }

    // =========================================================================
    // 1. Attitude Estimation via Complementary Filter
    // =========================================================================
    float ax = imu.accel_x_m_s2;
    float ay = imu.accel_y_m_s2;
    float az = imu.accel_z_m_s2;

    // Accelerometer pitch and roll (tilt angles)
    float rollAccDeg  = std::atan2(ay, az) * RAD_TO_DEG_F;
    float pitchAccDeg = std::atan2(-ax, std::sqrt(ay * ay + az * az)) * RAD_TO_DEG_F;

    // Gyroscope angular rate integration
    float rollGyro  = _rollDeg + (imu.gyro_x_dps * dt);
    float pitchGyro = _pitchDeg + (imu.gyro_y_dps * dt);
    _yawDeg        += (imu.gyro_z_dps * dt);

    // Normalize yaw to 0 - 360 degrees
    while (_yawDeg < 0.0f)   _yawDeg += 360.0f;
    while (_yawDeg >= 360.0f)_yawDeg -= 360.0f;

    // Complementary filter fusion
    constexpr float alpha = Config::ATTITUDE_FILTER_ALPHA;
    _rollDeg  = (alpha * rollGyro) + ((1.0f - alpha) * rollAccDeg);
    _pitchDeg = (alpha * pitchGyro) + ((1.0f - alpha) * pitchAccDeg);

    // Update IMU and Flight data structures
    imu.roll_deg  = _rollDeg;
    imu.pitch_deg = _pitchDeg;
    imu.yaw_deg   = _yawDeg;

    flight.roll_deg  = _rollDeg;
    flight.pitch_deg = _pitchDeg;
    flight.yaw_deg   = _yawDeg;

    // =========================================================================
    // 2. Altitude Estimation (VL53L0X with tilt correction)
    // =========================================================================
    if (tof.measurement_valid) {
        float rawDistCm = tof.distance_mm / 10.0f;

        // Correct for vehicle attitude tilt
        float rollRad  = _rollDeg * DEG_TO_RAD_F;
        float pitchRad = _pitchDeg * DEG_TO_RAD_F;
        float correctedAltCm = rawDistCm * std::cos(pitchRad) * std::cos(rollRad);

        // Low-pass filter altitude estimate
        _altitudeCm = (0.85f * _altitudeCm) + (0.15f * correctedAltCm);

        // Vertical velocity estimate (cm/s)
        float instVel = (_altitudeCm - _prevAltitudeCm) / dt;
        _verticalVel = (0.7f * _verticalVel) + (0.3f * instVel);
        _prevAltitudeCm = _altitudeCm;
    }

    altitudeCm = _altitudeCm;
    flight.vertical_velocity_cm_s = _verticalVel;

    // =========================================================================
    // 3. Horizontal Drift & Optical Flow Metric Velocity Estimation
    // =========================================================================
    if (flow.tracking_ok && _altitudeCm > 3.0f) {
        // Gyro rates in rad/s
        float gxRad = imu.gyro_x_dps * DEG_TO_RAD_F;
        float gyRad = imu.gyro_y_dps * DEG_TO_RAD_F;

        // Optical flow scale factor: radians per pixel
        float fovRad = Config::FLOW_FOV_RAD;
        float radPerPixel = fovRad / Config::FLOW_PIXELS_TOTAL;

        // Compensate for body rotation inducing apparent optical flow
        float rotDeltaX = (-gyRad * dt) / radPerPixel;
        float rotDeltaY = ( gxRad * dt) / radPerPixel;

        float transDeltaX = flow.delta_x - rotDeltaX;
        float transDeltaY = flow.delta_y - rotDeltaY;

        // Metric linear velocity: V = (trans_delta * rad_per_pixel * altitude_cm) / dt
        float instVelX = (transDeltaX * radPerPixel * _altitudeCm) / dt;
        float instVelY = (transDeltaY * radPerPixel * _altitudeCm) / dt;

        _filteredVelX = (0.8f * _filteredVelX) + (0.2f * instVelX);
        _filteredVelY = (0.8f * _filteredVelY) + (0.2f * instVelY);

        flow.velocity_x_cm_s = _filteredVelX;
        flow.velocity_y_cm_s = _filteredVelY;

        driftX = flow.delta_x;
        driftY = flow.delta_y;
    } else {
        flow.velocity_x_cm_s = 0.0f;
        flow.velocity_y_cm_s = 0.0f;
        driftX = 0.0f;
        driftY = 0.0f;
    }
}

} // namespace BeeBrain
