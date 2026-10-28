#include "telemetry.h"
#include <cstdio>
#include <algorithm>

namespace BeeBrain {

TelemetryPublisher::TelemetryPublisher()
    : _destIp(Config::PI_FORWARDER_IP), _port(Config::UDP_TELEMETRY_PORT) {}

bool TelemetryPublisher::init(const char* ssid,
                              const char* password,
                              const char* destinationIp,
                              uint16_t port) {
    _destIp = destinationIp;
    _port   = port;
    _sequenceNumber = 0;
    _packetsSent    = 0;
    _packetsDropped = 0;

    // Start Wi-Fi in Station mode
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    // Initialize UDP listener socket
    _udp.begin(_port);

    return true;
}

void TelemetryPublisher::maintainConnection() {
    uint32_t now = millis();
    if (now - _lastWifiCheckMs > 3000) {
        _lastWifiCheckMs = now;
        if (WiFi.status() != WL_CONNECTED) {
            // Non-blocking reconnect attempt
            WiFi.reconnect();
        }
    }
}

size_t TelemetryPublisher::serializeJSON(const FullTelemetrySnapshot& snap,
                                         char* buf,
                                         size_t maxLen) {
    // Quality must be clamped strictly within [0.0, 1.0] for Pydantic validation
    float quality = std::max(0.0f, std::min(1.0f, snap.optical_flow.quality));

    // Voltage must be non-negative
    float battV = std::max(0.0f, snap.battery.voltage_v);
    float soc   = std::max(0.0f, std::min(100.0f, snap.battery.state_of_charge_pct));

    // Format strict JSON conforming to TelemetryPacket (ConfigDict: extra="forbid")
    int written = snprintf(
        buf, maxLen,
        "{"
        "\"altitude_cm\":%.2f,"
        "\"drift_x\":%.2f,"
        "\"drift_y\":%.2f,"
        "\"battery_v\":%.2f,"
        "\"motor_pwm\":[%lu,%lu,%lu,%lu],"
        "\"link_ok\":%s,"
        "\"sensor_status\":{\"tof\":%s,\"optical_flow\":%s,\"imu\":%s,\"barometer\":%s},"
        "\"timestamp_s\":%.3f,"
        "\"sequence\":%lu,"
        "\"tof\":{\"distance_mm\":%.1f,\"signal_rate\":%.2f,\"measurement_valid\":%s},"
        "\"optical_flow\":{\"delta_x\":%.1f,\"delta_y\":%.1f,\"velocity_x_cm_s\":%.2f,\"velocity_y_cm_s\":%.2f,\"quality\":%.2f,\"tracking_ok\":%s},"
        "\"imu\":{\"roll_deg\":%.2f,\"pitch_deg\":%.2f,\"yaw_deg\":%.2f,\"gyro_x_dps\":%.2f,\"gyro_y_dps\":%.2f,\"gyro_z_dps\":%.2f,\"accel_x_m_s2\":%.2f,\"accel_y_m_s2\":%.2f,\"accel_z_m_s2\":%.2f},"
        "\"flight\":{\"target_altitude_cm\":%.1f,\"altitude_error_cm\":%.1f,\"vertical_velocity_cm_s\":%.2f,\"roll_deg\":%.2f,\"pitch_deg\":%.2f,\"yaw_deg\":%.2f,\"armed\":%s,\"flight_mode\":\"%s\"},"
        "\"battery\":{\"voltage_v\":%.2f,\"current_a\":%.2f,\"power_w\":%.2f,\"state_of_charge_pct\":%.1f,\"temperature_c\":%.1f},"
        "\"motors\":{\"pwm\":[%lu,%lu,%lu,%lu]},"
        "\"communications\":{\"link_ok\":%s,\"packet_rate_hz\":%.1f,\"latency_ms\":%.1f,\"rssi_dbm\":%.1f,\"packets_received\":%lu,\"packets_dropped\":%lu,\"packet_loss_pct\":%.1f},"
        "\"system\":{\"uptime_s\":%.2f,\"loop_rate_hz\":%.1f,\"processing_time_ms\":%.2f,\"error_count\":%lu,\"warning_count\":%lu}"
        "}\n",
        snap.altitude_cm,
        snap.drift_x,
        snap.drift_y,
        battV,
        static_cast<unsigned long>(snap.motor_pwm[0]),
        static_cast<unsigned long>(snap.motor_pwm[1]),
        static_cast<unsigned long>(snap.motor_pwm[2]),
        static_cast<unsigned long>(snap.motor_pwm[3]),
        snap.link_ok ? "true" : "false",
        snap.sensor_status.tof ? "true" : "false",
        snap.sensor_status.optical_flow ? "true" : "false",
        snap.sensor_status.imu ? "true" : "false",
        snap.sensor_status.barometer ? "true" : "false",
        snap.timestamp_s,
        static_cast<unsigned long>(snap.sequence),
        snap.tof.distance_mm,
        snap.tof.signal_rate,
        snap.tof.measurement_valid ? "true" : "false",
        snap.optical_flow.delta_x,
        snap.optical_flow.delta_y,
        snap.optical_flow.velocity_x_cm_s,
        snap.optical_flow.velocity_y_cm_s,
        quality,
        snap.optical_flow.tracking_ok ? "true" : "false",
        snap.imu.roll_deg,
        snap.imu.pitch_deg,
        snap.imu.yaw_deg,
        snap.imu.gyro_x_dps,
        snap.imu.gyro_y_dps,
        snap.imu.gyro_z_dps,
        snap.imu.accel_x_m_s2,
        snap.imu.accel_y_m_s2,
        snap.imu.accel_z_m_s2,
        snap.flight.target_altitude_cm,
        snap.flight.altitude_error_cm,
        snap.flight.vertical_velocity_cm_s,
        snap.flight.roll_deg,
        snap.flight.pitch_deg,
        snap.flight.yaw_deg,
        snap.flight.armed ? "true" : "false",
        snap.flight.flight_mode,
        battV,
        snap.battery.current_a,
        snap.battery.power_w,
        soc,
        snap.battery.temperature_c,
        static_cast<unsigned long>(snap.motors.pwm[0]),
        static_cast<unsigned long>(snap.motors.pwm[1]),
        static_cast<unsigned long>(snap.motors.pwm[2]),
        static_cast<unsigned long>(snap.motors.pwm[3]),
        snap.communications.link_ok ? "true" : "false",
        snap.communications.packet_rate_hz,
        snap.communications.latency_ms,
        snap.communications.rssi_dbm,
        static_cast<unsigned long>(snap.communications.packets_received),
        static_cast<unsigned long>(snap.communications.packets_dropped),
        snap.communications.packet_loss_pct,
        snap.system.uptime_s,
        snap.system.loop_rate_hz,
        snap.system.processing_time_ms,
        static_cast<unsigned long>(snap.system.error_count),
        static_cast<unsigned long>(snap.system.warning_count)
    );

    if (written < 0 || static_cast<size_t>(written) >= maxLen) {
        return 0;
    }
    return static_cast<size_t>(written);
}

bool TelemetryPublisher::publish(const FullTelemetrySnapshot& snap) {
    _sequenceNumber++;

    FullTelemetrySnapshot mutableSnap = snap;
    mutableSnap.sequence = _sequenceNumber;
    mutableSnap.timestamp_s = static_cast<float>(micros()) / 1000000.0f;
    mutableSnap.communications.link_ok = (WiFi.status() == WL_CONNECTED);
    mutableSnap.communications.rssi_dbm = getWifiRssiDbm();
    mutableSnap.communications.packets_received = _sequenceNumber;
    mutableSnap.communications.packets_dropped = _packetsDropped;

    size_t len = serializeJSON(mutableSnap, _jsonBuffer, sizeof(_jsonBuffer));
    if (len == 0) {
        _packetsDropped++;
        return false;
    }

    // 1. Send via USB CDC Serial
    Serial.write(reinterpret_cast<const uint8_t*>(_jsonBuffer), len);

    // 2. Send via Wi-Fi UDP (if connected)
    if (WiFi.status() == WL_CONNECTED) {
        if (_udp.beginPacket(_destIp, _port)) {
            _udp.write(reinterpret_cast<const uint8_t*>(_jsonBuffer), len);
            if (_udp.endPacket()) {
                _packetsSent++;
            } else {
                _packetsDropped++;
            }
        } else {
            _packetsDropped++;
        }
    } else {
        maintainConnection();
    }

    return true;
}

} // namespace BeeBrain
