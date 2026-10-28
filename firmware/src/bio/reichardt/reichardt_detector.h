#pragma once

#include <Arduino.h>
#include "include/system_config.h"

namespace BeeBrain {

class ReichardtDetector {
public:
    ReichardtDetector(float tauS = Config::REICHARDT_TAU_S);

    void reset();

    /**
     * @brief Process optical flow input through Hassenstein-Reichardt EMD model.
     * @param rawFlow Raw optical flow rate.
     * @param dt Sample interval in seconds.
     * @return Filtered bio-inspired motion signal.
     */
    float process(float rawFlow, float dt);

private:
    float _tauS;
    float _delayedSignal = 0.0f;
    float _prevRawSignal = 0.0f;
};

} // namespace BeeBrain
