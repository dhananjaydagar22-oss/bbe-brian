"""
Integration verification test for Bee-Brain firmware JSON serialization.
Validates that firmware output strictly complies with TelemetryPacket schema.
"""

import json
from dashboard.services.telemetry import TelemetryPacket

FIRMWARE_SAMPLE_JSON = """{
"altitude_cm":98.50,
"drift_x":0.12,
"drift_y":-0.04,
"battery_v":3.90,
"motor_pwm":[0,0,0,0],
"link_ok":true,
"sensor_status":{"tof":true,"optical_flow":true,"imu":true,"barometer":false},
"timestamp_s":14.825,
"sequence":1482,
"tof":{"distance_mm":985.0,"signal_rate":1.25,"measurement_valid":true},
"optical_flow":{"delta_x":1.2,"delta_y":-0.4,"velocity_x_cm_s":12.00,"velocity_y_cm_s":-4.00,"quality":0.85,"tracking_ok":true},
"imu":{"roll_deg":1.50,"pitch_deg":-0.80,"yaw_deg":45.20,"gyro_x_dps":0.02,"gyro_y_dps":-0.01,"gyro_z_dps":0.05,"accel_x_m_s2":0.15,"accel_y_m_s2":-0.08,"accel_z_m_s2":9.80},
"flight":{"target_altitude_cm":100.0,"altitude_error_cm":1.5,"vertical_velocity_cm_s":0.00,"roll_deg":1.50,"pitch_deg":-0.80,"yaw_deg":45.20,"armed":false,"flight_mode":"BENCH_TEST"},
"battery":{"voltage_v":3.90,"current_a":0.00,"power_w":0.00,"state_of_charge_pct":75.0,"temperature_c":25.0},
"motors":{"pwm":[0,0,0,0]},
"communications":{"link_ok":true,"packet_rate_hz":50.0,"latency_ms":1.2,"rssi_dbm":-55.0,"packets_received":1482,"packets_dropped":0,"packet_loss_pct":0.0},
"system":{"uptime_s":29.64,"loop_rate_hz":100.0,"processing_time_ms":1.80,"error_count":0,"warning_count":0}
}"""

def test_firmware_telemetry_schema():
    packet = TelemetryPacket.model_validate_json(FIRMWARE_SAMPLE_JSON)
    assert packet.altitude_cm == 98.50
    assert packet.drift_x == 0.12
    assert packet.drift_y == -0.04
    assert packet.battery_v == 3.90
    assert packet.motor_pwm == [0, 0, 0, 0]
    assert packet.link_ok is True
    assert packet.sensor_status.tof is True
    assert packet.sensor_status.optical_flow is True
    assert packet.sensor_status.imu is True
    assert packet.sensor_status.barometer is False
    assert packet.timestamp_s == 14.825
    assert packet.sequence == 1482
    assert packet.tof.distance_mm == 985.0
    assert packet.optical_flow.quality == 0.85
    assert packet.imu.roll_deg == 1.50
    assert packet.flight.flight_mode == "BENCH_TEST"
    assert packet.system.uptime_s == 29.64
    print("ALL ASSERTIONS PASSED: Firmware telemetry format strictly matches TelemetryPacket!")

if __name__ == "__main__":
    test_firmware_telemetry_schema()
