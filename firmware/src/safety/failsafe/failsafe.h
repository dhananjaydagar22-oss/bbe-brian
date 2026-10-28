#pragma once

#include <Arduino.h>
#include "include/system_config.h"
#include "include/telemetry_types.h"

namespace BeeBrain {

enum class FailsafeState {
    BENCH_TEST,
    ARMED_BENCH,
    DISARMED,
    EMERGENCY_DISARM,
    LOW_BATTERY,
    SENSOR_FAULT
};

class FailsafeManager {
public:
    FailsafeManager();

    /**
     * @brief Evaluate all safety criteria and update system flight mode and armed state.
     * @param status Current sensor status flags.
     * @param battery Current battery telemetry.
     * @param loomingThreat True if LGMD collision detector triggered.
     * @param flight Output flight telemetry to update with mode and armed flag.
     * @param system Output system diagnostics to update with error/warning counts.
     */
    void update(const SensorStatusData& status,
                const BatteryData& battery,
                bool loomingThreat,
                FlightData& flight,
                SystemData& system);

    /**
     * @brief Request explicit arming for bench test mode.
     * @return true if allowed by safety checks.
     */
    bool requestArmBench();

    /**
     * @brief Request manual disarm.
     */
    void requestDisarm();

    /**
     * @brief Trigger emergency shutdown.
     */
    void triggerEmergencyShutdown();

    FailsafeState getState() const { return _state; }
    bool isArmed() const { return _isArmed; }
    const char* getStateString() const;

private:
    FailsafeState _state    = FailsafeState::BENCH_TEST;
    bool          _isArmed  = false;
    uint32_t      _errors   = 0;
    uint32_t      _warnings = 0;
};

} // namespace BeeBrain
