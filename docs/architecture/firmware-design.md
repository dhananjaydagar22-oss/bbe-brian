# Bee-Brain ESP32-S3 Firmware Architecture & Design Specification

## 1. Executive Summary

This document specifies the software architecture, hardware interfacing, coordinate frames, FreeRTOS task scheduling, state estimation, bio-inspired algorithms, and communication protocol for the **Bee-Brain** embedded firmware running on the **Seeed Studio XIAO ESP32-S3**.

The firmware is designed for **Stage 1 (Reliable Data Acquisition)** of the Bee-Brain project roadmap:
- Read and validate physical sensors independently: **MPU6050** (6-DOF IMU), **ADNS3080** (Optical Flow), and **VL53L0X** (Time-of-Flight rangefinder).
- Implement deterministic FreeRTOS dual-core task scheduling (100 Hz sensor acquisition & state estimation on Core 1; 20–50 Hz telemetry serialization and Wi-Fi UDP streaming on Core 0).
- Guarantee explicit units, coordinate systems (ROS REP-103 FLU), microsecond timestamps, monotonic sequence numbers, and granular sensor validity flags.
- Maintain dual-stream transmission: USB-CDC Serial (115200 baud) for tethered bench debugging and Wi-Fi UDP for streaming to the Raspberry Pi 3 Model B+ companion computer.
- Enforce strict telemetry schema compliance with the ground station dashboard and ROS 2 ingest pipelines.
- Isolate bench actuators (SG90 servo test actuator) from unverified flight systems, adhering to strict bench safety and failsafe protocols.

---

## 2. Hardware Architecture & Pin Assignments

### 2.1 Microcontroller Target
- **Board**: Seeed Studio XIAO ESP32-S3
- **SoC**: Espressif ESP32-S3 (Dual-core 32-bit Xtensa LX7 @ 240 MHz, 512 KB SRAM, 8 MB Flash, 2.4 GHz 802.11b/g/n Wi-Fi, BLE 5.0)
- **Carrier**: Female header perfboard carrier socket

### 2.2 Pin Allocation Matrix

| Subsystem | Peripheral Pin | XIAO ESP32-S3 Pin | GPIO Pin | Bus / Signal Function | Electrical Notes |
|---|---|---|---|---|---|
| **I2C Bus** | SDA | `D4` | `GPIO 5` | I2C Data line | Shared by MPU6050 & VL53L0X (4.7kΩ pull-up to 3.3V) |
| | SCL | `D5` | `GPIO 6` | I2C Clock line | Shared by MPU6050 & VL53L0X (400 kHz Fast Mode) |
| **MPU6050 (IMU)** | I2C Addr AD0 | - | - | Tied to GND | I2C Address `0x68` |
| | INT (Optional) | `D1` | `GPIO 2` | Data Ready Interrupt | Active High / Open-Drain |
| **VL53L0X (ToF)** | XSHUT | - | - | Hardware Shutdown | Tied to 3.3V (Address `0x29`) |
| **ADNS3080 (Flow)** | SPI SCK | `D8` | `GPIO 7` | SPI Clock | Up to 2 MHz, SPI Mode 3 (CPOL=1, CPHA=1) |
| | SPI MISO | `D9` | `GPIO 8` | SPI Master-In Slave-Out | Driven by ADNS3080 |
| | SPI MOSI | `D10` | `GPIO 9` | SPI Master-Out Slave-In | Commands and register addresses |
| | SPI CS | `D3` | `GPIO 4` | Chip Select (Active Low) | Manual software GPIO assertion |
| | RESET | `D2` | `GPIO 3` | Hardware Reset | High pulse (>10 µs) at power-on |
| **Battery Monitor** | Divider In | `A0` | `GPIO 1` | ADC1 Channel 0 | 1S LiPo voltage divider (e.g. 2:1 ratio, 0–4.2V mapped to 0–2.1V) |
| **Bench Actuator** | SG90 PWM | `D7` | `GPIO 44` | LEDC PWM Channel 4 | 50 Hz PWM (1.0 ms – 2.0 ms pulse width) for pan-tilt test rig |
| **Bench Motor PWM**| Motor 1 (FL) | - | `GPIO 41` | LEDC PWM Channel 0 | 500 Hz – 20 kHz PWM (Bench test mode only, disarmed by default) |
| | Motor 2 (FR) | - | `GPIO 42` | LEDC PWM Channel 1 | Coreless MOSFET drive |
| | Motor 3 (RR) | - | `GPIO 38` | LEDC PWM Channel 2 | Coreless MOSFET drive |
| | Motor 4 (RL) | - | `GPIO 39` | LEDC PWM Channel 3 | Coreless MOSFET drive |
| **Serial / USB** | USB CDC / UART | `D6` | `GPIO 43` | Native USB CDC / TXD | 115200 baud debug console & telemetry |

---

## 3. Coordinate Frames, Conventions & Physical Units

Adhering to standard robotics practices (**ROS REP-103**):

```text
       Forward (+X)
           ^
           |
  (+Y) <---+---- (Body Frame Origin)
  Left     |
          (+Z) Up (Right-hand rule: Roll about +X, Pitch about +Y, Yaw about +Z)
```

### 3.1 Sensor Reference Frames & Mappings
1. **Body Frame $B$**:
   - $+X_B$: Points directly forward towards drone front.
   - $+Y_B$: Points leftward.
   - $+Z_B$: Points upward (normal to top surface).
2. **MPU6050 Accelerometer & Gyroscope**:
   - Accelerometer Output: $a_x, a_y, a_z$ in SI units ($m/s^2$).
   - Gyroscope Output: $\omega_x, \omega_y, \omega_z$ in degrees per second ($^\circ/s$) and radians per second ($rad/s$).
   - Sensor orientation: Mounted flat, aligned with Body Frame ($+X_{sensor} = +X_B$, $+Y_{sensor} = +Y_B$, $+Z_{sensor} = +Z_B$).
   - Static Level Reading: $a_x \approx 0.0$, $a_y \approx 0.0$, $a_z \approx +9.81\ m/s^2$.
3. **VL53L0X Time-of-Flight**:
   - Measurement Axis: Pointing downward along $-Z_B$.
   - Raw Output: Direct optical distance $d_{raw}$ in millimeters ($mm$) and centimeters ($cm$).
   - Altitude Correction: Ground clearance accounting for vehicle attitude tilt:
     $$h_{true} = d_{raw} \cdot \cos(\theta_{pitch}) \cdot \cos(\phi_{roll})$$
4. **ADNS3080 Optical Flow**:
   - Measurement Axis: Facing downward along $-Z_B$.
   - Output: Surface displacement counts $\Delta x_{flow}, \Delta y_{flow}$ per sample window $\Delta t$.
   - Surface Quality: SQUAL ($0 \dots 255$, normalized to $0.0 \dots 1.0$).
   - Rotational Gyro Compensation & Metric Velocity Conversion:
     $$v_{x} = \left(\frac{\Delta x_{flow} \cdot 2 \cdot \tan\left(\frac{FOV_x}{2}\right) \cdot h}{N_{pixels} \cdot \Delta t}\right) - (h \cdot \omega_y)$$
     $$v_{y} = \left(\frac{\Delta y_{flow} \cdot 2 \cdot \tan\left(\frac{FOV_y}{2}\right) \cdot h}{N_{pixels} \cdot \Delta t}\right) + (h \cdot \omega_x)$$
     *(where $h$ is ground distance from VL53L0X, and $\omega_x, \omega_y$ are gyro angular rates in $rad/s$)*.

---

## 4. FreeRTOS Dual-Core Concurrency Architecture

The ESP32-S3 features two Xtensa LX7 cores running at 240 MHz. To prevent network latency and serialization overhead from introducing jitter into real-time sensor sampling, responsibilities are split across cores:

```text
+--------------------------------------------------------------------------------+
|                             ESP32-S3 DUAL-CORE MODEL                           |
+--------------------------------------------------------------------------------+
|  CORE 1 (High Priority Real-Time)            CORE 0 (Communication & I/O)      |
|                                                                                |
|  [ Sensor & Estimation Task ]               [ Telemetry & Comms Task ]        |
|  - Period: 10.0 ms (100 Hz)                 - Period: 20.0 ms - 50.0 ms       |
|  - MPU6050 I2C Burst Read                   - Snapshot Sensor & State Buffer   |
|  - ADNS3080 SPI Burst Read                  - Format Strict JSON String        |
|  - VL53L0X ToF Range Acquisition            - Transmit Serial (USB-CDC)        |
|  - Attitude Complementary Filter            - Transmit UDP to Pi (Port 8888)   |
|  - Altitude & Velocity Estimation           - Wi-Fi Connection Watchdog        |
|  - Reichardt EMD & LGMD Processing          - Heartbeat & Diagnostics Counters |
|  - SG90 Bench Servo PWM Update                                                 |
|                     \                              ^                           |
|                      \                            /                            |
|                       v                          /                             |
|              +------------------------------------------+                      |
|              | Thread-Safe State Snapshot (FreeRTOS)    |                      |
|              | Double-buffer with Mutex / Spinlock      |                      |
|              +------------------------------------------+                      |
+--------------------------------------------------------------------------------+
```

### 4.1 Task Priority Configuration
- `SensorTask` on **Core 1**: Priority `5` (High). Uses `vTaskDelayUntil()` for deterministic 100 Hz scheduling.
- `TelemetryTask` on **Core 0**: Priority `2` (Medium). Handles JSON string formatting, UDP socket send, and USB UART writes.
- `FailsafeMonitor` on **Core 1**: Embedded inside SensorTask cycle. Evaluates timeouts, sensor dropouts, and low voltage every 10 ms.

---

## 5. Sensor Driver Implementations & Error Recovery

### 5.1 MPU6050 (6-DOF IMU)
- **Interface**: I2C (400 kHz), Address `0x68`.
- **Registers**: `PWR_MGMT_1` (0x6B), `CONFIG` (0x1A), `GYRO_CONFIG` (0x1B), `ACCEL_CONFIG` (0x1C), `ACCEL_XOUT_H` (0x3B).
- **Initialization**: Wake device (clear sleep bit), set Digital Low Pass Filter (DLPF) to 42 Hz bandwidth, configure Accel $\pm 4g$ (8192 LSB/g) and Gyro $\pm 500^\circ/s$ (65.5 LSB/(°/s)).
- **Zero-Rate Calibration**: On boot (while resting stationary on bench), samples 200 readings to calculate gyroscope zero-bias offsets ($b_{\omega x}, b_{\omega y}, b_{\omega z}$).
- **Fault Detection**: Validates WHO_AM_I register (expects `0x68`). Tracks communication timeouts. If 5 consecutive I2C reads fail, sets `sensor_status.imu = false` and raises warning count.

### 5.2 VL53L0X (Time-of-Flight Laser Ranger)
- **Interface**: I2C (400 kHz), Address `0x29`.
- **Operation Mode**: Continuous single-ranging mode or high-accuracy timed ranging.
- **Data Validation**: Checks range status flag (0 = Valid Range, 1 = Sigma Fail, 2 = Signal Fail, 4 = Phase Fail). Only flags `measurement_valid = true` if status == 0 and reading is between 30 mm and 2000 mm.
- **Altitude Outlier Rejection**: Rejects single-frame discontinuities exceeding physical velocity bounds ($> 5.0\ m/s$).

### 5.3 ADNS3080 (Optical Flow)
- **Interface**: SPI Mode 3, 2 MHz clock, Chip Select GPIO 4.
- **Registers**: `PRODUCT_ID` (0x00, expects 0x17), `MOTION` (0x02), `DELTA_X` (0x03), `DELTA_Y` (0x04), `SQUAL` (0x05).
- **Motion Burst Protocol**: Asserts CS, writes Motion Burst command (`0x50`), waits 75 µs turnaround time, reads motion byte, delta X, delta Y, and surface quality in a single SPI transaction.
- **Tracking Quality Assessment**: SQUAL values below 15 indicate poor surface texture or out-of-focus optics; sets `tracking_ok = false`.

---

## 6. State Estimation & Attitude Filtering

### 6.1 Attitude Estimation (Complementary Filter)
Given the lack of a magnetometer on the MPU6050, the firmware implements an attitude observer estimating Roll ($\phi$) and Pitch ($\theta$):
1. **Accelerometer Incline**:
   $$\phi_{acc} = \arctan2(a_y, a_z)$$
   $$\theta_{acc} = \arctan2(-a_x, \sqrt{a_y^2 + a_z^2})$$
2. **Gyroscope Integration**:
   $$\phi_{k} = \phi_{k-1} + \omega_x \cdot \Delta t$$
   $$\theta_{k} = \theta_{k-1} + \omega_y \cdot \Delta t$$
   $$\psi_{k} = \psi_{k-1} + \omega_z \cdot \Delta t \quad (\text{Relative Yaw})$$
3. **Complementary Fusion ($\alpha = 0.98$)**:
   $$\phi = \alpha \cdot (\phi + \omega_x \cdot \Delta t) + (1 - \alpha) \cdot \phi_{acc}$$
   $$\theta = \alpha \cdot (\theta + \omega_y \cdot \Delta t) + (1 - \alpha) \cdot \theta_{acc}$$

---

## 7. Bio-Inspired Perception Modules

### 7.1 Hassenstein-Reichardt Elementary Motion Detector (EMD)
Implements temporal motion correlation inspired by insect compound eyes:
- Two spatial input channels from optical flow displacement history.
- First-order low-pass temporal delay filter: $s_{delayed}(t) = s_{delayed}(t-1) + \frac{\Delta t}{\tau} (s(t) - s_{delayed}(t-1))$.
- Cross-multiplication of delayed channel with direct adjacent channel:
  $$EMD_{forward} = s_1(t) \cdot s_{2, delayed}(t) - s_2(t) \cdot s_{1, delayed}(t)$$
- Smooths jerky pixel motion and filters high-frequency sensor noise.

### 7.2 Lobula Giant Movement Detector (LGMD)
Models the collision-detection visual neuron found in locusts:
- Tracks rate of visual expansion and proximity closing speed using downward ToF distance:
  $$v_{approach} = -\frac{h_k - h_{k-1}}{\Delta t}$$
- If $v_{approach} > V_{threshold}$ (looming impact imminent during bench descent test), LGMD triggers an emergency disarm state and warning flag.

---

## 8. Actuator Subsystem & Bench Test Protocol

### 8.1 Safety Rules & Non-Flight Baseline
- **Bench Test Rule**: Do NOT treat the bench prototype as a flight-ready drone.
- **SG90 Servo**: Bench actuator test only. Generates 50 Hz PWM on GPIO 44 (1.0 ms to 2.0 ms pulse width).
- **Coreless DC Motor PWM**: 4 channels configured via ESP32-S3 LEDC peripheral. All channels initialized to 0 duty cycle. Armed state is locked out unless explicit bench override command is received via serial.

---

## 9. Telemetry Protocol & Wire Format

The firmware outputs telemetry packets formatted as single-line JSON terminated with a newline (`\n`). Every packet strictly conforms to the project specification and passes Pydantic validation with `ConfigDict(extra="forbid")`.

### 9.1 Unified JSON Packet Schema
```json
{
  "altitude_cm": 98.5,
  "drift_x": 0.12,
  "drift_y": -0.04,
  "battery_v": 3.90,
  "motor_pwm": [0, 0, 0, 0],
  "link_ok": true,
  "sensor_status": {
    "tof": true,
    "optical_flow": true,
    "imu": true,
    "barometer": false
  },
  "timestamp_s": 14.825,
  "sequence": 1482,
  "tof": {
    "distance_mm": 985.0,
    "signal_rate": 1.25,
    "measurement_valid": true
  },
  "optical_flow": {
    "delta_x": 1.2,
    "delta_y": -0.4,
    "velocity_x_cm_s": 12.0,
    "velocity_y_cm_s": -4.0,
    "quality": 0.85,
    "tracking_ok": true
  },
  "imu": {
    "roll_deg": 1.5,
    "pitch_deg": -0.8,
    "yaw_deg": 45.2,
    "gyro_x_dps": 0.02,
    "gyro_y_dps": -0.01,
    "gyro_z_dps": 0.05,
    "accel_x_m_s2": 0.15,
    "accel_y_m_s2": -0.08,
    "accel_z_m_s2": 9.80
  },
  "flight": {
    "target_altitude_cm": 100.0,
    "altitude_error_cm": 1.5,
    "vertical_velocity_cm_s": 0.0,
    "roll_deg": 1.5,
    "pitch_deg": -0.8,
    "yaw_deg": 45.2,
    "armed": false,
    "flight_mode": "BENCH_TEST"
  },
  "battery": {
    "voltage_v": 3.90,
    "current_a": 0.0,
    "power_w": 0.0,
    "state_of_charge_pct": 75.0,
    "temperature_c": 25.0
  },
  "motors": {
    "pwm": [0, 0, 0, 0]
  },
  "communications": {
    "link_ok": true,
    "packet_rate_hz": 50.0,
    "latency_ms": 1.2,
    "rssi_dbm": -55.0,
    "packets_received": 1482,
    "packets_dropped": 0,
    "packet_loss_pct": 0.0
  },
  "system": {
    "uptime_s": 29.64,
    "loop_rate_hz": 100.0,
    "processing_time_ms": 1.8,
    "error_count": 0,
    "warning_count": 0
  }
}
```

---

## 10. Stage 1 Verification & Physical Testing Checklist

1. **Power & Bus Verification**:
   - Power XIAO ESP32-S3 via USB-C. Confirm 3.3V rail stability with multimeter.
   - Run I2C Scanner routine. Confirm presence of device at `0x68` (MPU6050) and `0x29` (VL53L0X).
2. **MPU6050 Verification**:
   - Place board flat on level surface. Confirm $a_z \approx +9.81\ m/s^2$ and $a_x, a_y \approx 0.0\ m/s^2$.
   - Tilt 90° roll and pitch. Verify angle estimation reflects sign and magnitude correctly.
3. **VL53L0X Verification**:
   - Point downward at known distances (10 cm, 20 cm, 50 cm, 100 cm).
   - Check `distance_mm` against tape measure. Confirm status invalid when sensor is occluded or out of range.
4. **ADNS3080 Verification**:
   - Slide textured surface under sensor. Observe `delta_x` and `delta_y` counts increment and decrement.
   - Verify `quality` reads $> 0.15$ over textured paper and drops over blank reflective surfaces.
5. **Servo Bench Test**:
   - Confirm SG90 sweeps smoothly from 0° to 180° on GPIO 44 without power drops or resets.
6. **UDP Streaming & Logging**:
   - Start listener on Raspberry Pi / Laptop (`nc -u -l 8888`). Confirm packets arrive at steady 20–50 Hz with zero sequence drops.
