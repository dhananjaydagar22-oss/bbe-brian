#pragma once

#include <Arduino.h>
#include "include/system_config.h"

namespace BeeBrain {

class LGMDDetector {
public:
    LGMDDetector(float velocityThresholdCmS = Config::LGMD_COLLISION_VEL_THRESH);

    void reset();

    /**
     * @brief Update looming collision model with current distance and optical flow expansion.
     * @param distanceCm Current range reading from downward ToF.
     * @param dt Elapsed time in seconds.
     * @return True if imminent looming collision detected.
     */
    bool update(float distanceCm, float dt);

    /**
     * @brief Return true if looming emergency trigger is active.
     */
    bool isLoomingThreatDetected() const { return _threatDetected; }

    /**
     * @brief Get calculated approach velocity (negative indicates approaching).
     */
    float getApproachVelocityCmS() const { return _approachVelocity; }

private:
    float _thresholdCmS;
    float _prevDistanceCm    = 0.0f;
    float _approachVelocity  = 0.0f;
    bool  _threatDetected    = false;
    bool  _firstUpdate       = true;
};

} // namespace BeeBrain
