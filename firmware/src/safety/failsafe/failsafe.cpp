#include "failsafe.h"
#include <cstring>

namespace BeeBrain {

FailsafeManager::FailsafeManager() {
    _state    = FailsafeState::BENCH_TEST;
    _isArmed  = false;
    _errors   = 0;
    _warnings = 0;
}

const char* FailsafeManager::getStateString() const {
    switch (_state) {
        case FailsafeState::BENCH_TEST:       return "BENCH_TEST";
        case FailsafeState::ARMED_BENCH:      return "ARMED_BENCH";
        case FailsafeState::DISARMED:         return "DISARMED";
        case FailsafeState::EMERGENCY_DISARM: return "EMERGENCY_DISARM";
        case FailsafeState::LOW_BATTERY:      return "LOW_BATTERY";
        case FailsafeState::SENSOR_FAULT:     return "SENSOR_FAULT";
        default:                              return "UNKNOWN";
    }
}

void FailsafeManager::update(const SensorStatusData& status,
                             const BatteryData& battery,
                             bool loomingThreat,
                             FlightData& flight,
                             SystemData& system) {
    // 1. Check Looming Collision
    if (loomingThreat) {
        _state = FailsafeState::EMERGENCY_DISARM;
        _isArmed = false;
        _warnings++;
    }

    // 2. Check Critical Low Battery
    if (battery.voltage_v > 0.5f && battery.voltage_v <= Config::BATT_VOLTAGE_MIN_V) {
        _state = FailsafeState::LOW_BATTERY;
        _isArmed = false;
        _errors++;
    } else if (battery.voltage_v > 0.5f && battery.voltage_v <= Config::BATT_VOLTAGE_WARN_V) {
        _warnings++;
    }

    // 3. Check Critical Sensor Health (IMU is primary attitude reference)
    if (!status.imu) {
        _state = FailsafeState::SENSOR_FAULT;
        _isArmed = false;
        _errors++;
    }

    // Update flight struct
    flight.armed = _isArmed;
    strncpy(flight.flight_mode, getStateString(), sizeof(flight.flight_mode) - 1);
    flight.flight_mode[sizeof(flight.flight_mode) - 1] = '\0';

    // Update system diagnostic counters
    system.error_count   = _errors;
    system.warning_count = _warnings;
}

bool FailsafeManager::requestArmBench() {
    if (_state == FailsafeState::EMERGENCY_DISARM ||
        _state == FailsafeState::LOW_BATTERY ||
        _state == FailsafeState::SENSOR_FAULT) {
        return false;
    }

    _state   = FailsafeState::ARMED_BENCH;
    _isArmed = true;
    return true;
}

void FailsafeManager::requestDisarm() {
    _state   = FailsafeState::DISARMED;
    _isArmed = false;
}

void FailsafeManager::triggerEmergencyShutdown() {
    _state   = FailsafeState::EMERGENCY_DISARM;
    _isArmed = false;
    _errors++;
}

} // namespace BeeBrain
