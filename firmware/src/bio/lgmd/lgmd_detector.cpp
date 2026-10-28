#include "lgmd_detector.h"

namespace BeeBrain {

LGMDDetector::LGMDDetector(float velocityThresholdCmS)
    : _thresholdCmS(velocityThresholdCmS) {
    reset();
}

void LGMDDetector::reset() {
    _prevDistanceCm   = 0.0f;
    _approachVelocity = 0.0f;
    _threatDetected   = false;
    _firstUpdate      = true;
}

bool LGMDDetector::update(float distanceCm, float dt) {
    if (dt <= 0.0001f || distanceCm <= 0.0f) {
        return _threatDetected;
    }

    if (_firstUpdate) {
        _prevDistanceCm = distanceCm;
        _firstUpdate = false;
        return false;
    }

    // Approach velocity: rate of change of distance (negative means closing in)
    float instVel = (distanceCm - _prevDistanceCm) / dt;
    _prevDistanceCm = distanceCm;

    // Filtered closing velocity
    _approachVelocity = (0.7f * _approachVelocity) + (0.3f * instVel);

    // If approaching faster than threshold and close to obstacle (< 30 cm)
    if (_approachVelocity < _thresholdCmS && distanceCm < 30.0f) {
        _threatDetected = true;
    } else {
        _threatDetected = false;
    }

    return _threatDetected;
}

} // namespace BeeBrain
