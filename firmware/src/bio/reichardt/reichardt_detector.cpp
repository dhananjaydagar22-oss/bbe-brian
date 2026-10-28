#include "reichardt_detector.h"
#include <algorithm>

namespace BeeBrain {

ReichardtDetector::ReichardtDetector(float tauS)
    : _tauS(tauS) {
    reset();
}

void ReichardtDetector::reset() {
    _delayedSignal = 0.0f;
    _prevRawSignal = 0.0f;
}

float ReichardtDetector::process(float rawFlow, float dt) {
    if (dt <= 0.0001f) {
        return rawFlow;
    }

    // 1. First-order low-pass temporal delay filter
    float alpha = dt / (_tauS + dt);
    float currentDelayed = _delayedSignal + alpha * (rawFlow - _delayedSignal);

    // 2. Cross-multiplication of delayed and non-delayed channels (EMD model)
    // Left-to-right minus Right-to-left correlation
    float emdOutput = (rawFlow * _delayedSignal) - (currentDelayed * _prevRawSignal);

    // Update state
    _delayedSignal = currentDelayed;
    _prevRawSignal = rawFlow;

    return emdOutput;
}

} // namespace BeeBrain
