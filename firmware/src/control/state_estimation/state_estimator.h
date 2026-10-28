#pragma once

#include <Arduino.h>
#include "include/system_config.h"
#include "include/telemetry_types.h"

namespace BeeBrain {

class StateEstimator {
public:
    StateEstimator();

    /**
     * @brief Reset estimator state, attitude angles, and filter buffers.
     */
    void reset();

    /**
     * @brief Update attitude, altitude, and horizontal drift estimates.
     * @param imu IMU measurements (updated with computed roll, pitch, yaw).
     * @param tof Downward ToF measurement.
     * @param flow Optical flow measurement (updated with metric velocities).
     * @param flight Flight state structure.
     * @param altitudeCm Legacy altitude output in cm.
     * @param driftX Legacy drift X output.
     * @param driftY Legacy drift Y output.
     * @param dt Sampling time delta in seconds.
     */
    void update(IMUData& imu,
                const ToFData& tof,
                OpticalFlowData& flow,
                FlightData& flight,
                float& altitudeCm,
                float& driftX,
                float& driftY,
                float dt);

    float getEstimatedRollDeg() const { return _rollDeg; }
    float getEstimatedPitchDeg() const { return _pitchDeg; }
    float getEstimatedYawDeg() const { return _yawDeg; }
    float getEstimatedAltitudeCm() const { return _altitudeCm; }

private:
    float _rollDeg       = 0.0f;
    float _pitchDeg      = 0.0f;
    float _yawDeg        = 0.0f;
    float _altitudeCm    = 0.0f;
    float _prevAltitudeCm= 0.0f;
    float _verticalVel   = 0.0f;

    float _filteredVelX  = 0.0f;
    float _filteredVelY  = 0.0f;

    static constexpr float DEG_TO_RAD_F = 0.017453292519943295f;
    static constexpr float RAD_TO_DEG_F = 57.29577951308232f;
};

} // namespace BeeBrain
