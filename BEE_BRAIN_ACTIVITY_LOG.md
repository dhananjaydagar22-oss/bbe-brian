# Bee-Brain Project Activity & Audit Log

## Log Entries

### 2026-09-24 - Repository Restructuring & Architectural Scaffold Baseline
- **Developer / Agent**: Antigravity
- **Branch**: `dashboard-dev`
- **Purpose**: Converted repository from full implementation code into a clean, modular development scaffold and architectural baseline.
- **Affected Subsystems**:
  - `firmware/src/` (sensors, control, bio, communication, safety, main.cpp)
  - `dashboard/` (app.py, services, database, mapping, ui)
  - `simulation/` (controllers, worlds, sensors, algorithms)
  - `cad/` (scripts, step, stl, README.md)
  - `docs/` (hardware, research, telemetry, architecture)
  - `tests/` (integration, test_data)
- **Actions Executed**:
  1. Converted implementation source files (`.cpp`, `.h`, `.py`) to single-line architectural placeholder comments describing module responsibilities.
  2. Preserved configuration files (`firmware/platformio.ini`, `dashboard/requirements.txt`, `simulation/controllers/drone_controller/requirements.txt`, `.gitignore`, `LICENSE`).
  3. Preserved documentation contracts (`docs/hardware/pin-mapping.md`, `docs/research/research-notes.md`, `docs/telemetry/telemetry-spec.md`, `docs/README.md`, `README.md`).
  4. Preserved CAD asset files under `cad/step/frame_v1.step` and `cad/stl/frame_v1.stl`.
  5. Created `.gitkeep` markers in empty scaffold directories (`firmware/include`, `firmware/lib`, `firmware/test`, `simulation/algorithms`, `simulation/sensors`, `simulation/protos`, `dashboard/static`, `dashboard/templates`, `dashboard/tests`, `tests/test_data`).
  6. Removed top-level duplicate files (`cad/frame_v1.step`, `cad/frame_v1.stl`, `dashboard/fake_data_generator.py`, `docs/pin-mapping.md`, `docs/research-notes.md`, `docs/telemetry-spec.md`, `simulation/protos` file artifact).
  7. Purged Python bytecode caches (`dashboard/services/__pycache__/`).
  8. Created `BEE_BRAIN_PROJECT_CONTEXT.md` and initialized `BEE_BRAIN_ACTIVITY_LOG.md`.
- **Validation**: Verified folder tree hierarchy, checked git status, confirmed exact single-line placeholders, and validated zero implementation logic leaks in source files.

### 2026-09-25 - Telemetry Service & Packet Implementation
- **Developer / Agent**: Antigravity
- **Component / File Changed**: `dashboard/services/telemetry.py`
- **What Was Implemented**:
  - `SensorStatus` and `TelemetryPacket` Pydantic models for strict telemetry payload validation matching `docs/telemetry/telemetry-spec.md`.
  - `TelemetryService` class managing a rolling telemetry history buffer (`deque`, `maxlen=500`).
  - `add_packet(raw_json: str)` method parsing and validating JSON input into `TelemetryPacket` objects and raising `ValueError` on malformed/invalid payloads.
  - `get_latest()` method returning the most recent `TelemetryPacket` or `None`.
  - `get_history()` method returning all buffered packets as a `list[TelemetryPacket]`.
- **Important Design Decisions**:
  - Configured `ConfigDict(extra="forbid")` on Pydantic models to strictly enforce the specification contract and reject any undocumented fields.
  - Exception handling converts parsing and schema validation failures into descriptive `ValueError` exceptions.
- **Tests & Validation Performed**:
  - Verified packet parsing, type validation, schema enforcement (rejecting missing or extra fields), error handling for malformed JSON, and deque `maxlen=500` overflow behavior via execution of Python test scripts.
- **Current Status**: Complete and operational.

### 2026-09-25 - Fake Telemetry Generator
- **Developer / Agent**: Antigravity
- **Component / File Changed**: `dashboard/services/fake_data_generator.py`
- **What Was Implemented**:
  - Added `FakeTelemetryGenerator` with stateful simulated telemetry.
  - Added `QTimer`-based periodic generation via `start(callback, interval_ms)` and `stop()` methods.
  - Simulated hover stability, natural battery drain, slight drift, realistic motor PWM variations, and occasional auto-recovering dropouts for communication link and sensors (ToF, Optical Flow).
- **Important Design Decisions**:
  - Leveraged `TelemetryPacket` directly to ensure the generated payloads comply perfectly with the specification.
- **Tests & Validation Performed**:
  - Confirmed generated telemetry strictly adheres to `TelemetryPacket` schema constraint validations.
  - Verified syntax, logic, and dependency imports without executing hardware.
- **Current Status**: Fake telemetry source ready for dashboard UI development.

### 2026-10-28 - Stage 1 ESP32-S3 Firmware Architecture & Implementation
- **Developer / Agent**: Antigravity
- **Component / File Changed**: `firmware/` (all modules), `docs/architecture/firmware-design.md`, `tests/test_firmware_telemetry_schema.py`
- **What Was Implemented**:
  - Comprehensive firmware architecture specification covering hardware pinout, coordinate systems (ROS REP-103 FLU), FreeRTOS dual-core model, sensor pipelines, attitude estimation, bio-inspired filters, failsafe safety, telemetry serialization, and validation procedures (`docs/architecture/firmware-design.md`).
  - Pin mapping & system configuration definitions (`firmware/include/pin_config.h`, `firmware/include/system_config.h`, `firmware/include/telemetry_types.h`).
  - Physical sensor drivers: MPU6050 6-DOF IMU (I2C address 0x68, register burst read, gyro bias auto-calibration), VL53L0X downward ToF (I2C address 0x29, range validation), ADNS3080 downward optical flow (SPI Mode 3, 2 MHz, motion burst & surface quality).
  - Unified sensor manager (`firmware/src/sensors/sensor_manager.h/.cpp`) coordinating shared I2C bus, SPI, and ADC battery monitoring.
  - State estimator (`firmware/src/control/state_estimation/state_estimator.h/.cpp`) implementing complementary filter attitude estimation (roll, pitch, yaw), tilt-corrected altitude estimation, and gyro-compensated metric optical flow velocity estimation.
  - Modular PID controller (`firmware/src/control/pid/pid.h/.cpp`) with anti-windup clamping and derivative filtering.
  - Motor mixer and bench actuator driver (`firmware/src/control/motor_mixer/motor_mixer.h/.cpp`) driving SG90 servo via 50 Hz PWM on GPIO 44 and maintaining strict 0-PWM motor lockout in bench mode.
  - Bio-inspired perception modules: Hassenstein-Reichardt elementary motion detector (`firmware/src/bio/reichardt/reichardt_detector.h/.cpp`) and Lobula Giant Movement Detector looming collision model (`firmware/src/bio/lgmd/lgmd_detector.h/.cpp`).
  - Failsafe safety state machine (`firmware/src/safety/failsafe/failsafe.h/.cpp`) evaluating low voltage, sensor silence, and looming threats.
  - Telemetry publisher (`firmware/src/communication/telemetry/telemetry.h/.cpp`) serializing strict JSON conforming to `TelemetryPacket` (Pydantic `extra="forbid"` compliant) and streaming over USB-CDC Serial and Wi-Fi UDP on port 8888.
  - FreeRTOS dual-core entry point (`firmware/src/main.cpp`) pinning deterministic 100 Hz sensor loop to Core 1 and 50 Hz telemetry loop to Core 0 with critical section concurrency protection.
- **Important Design Decisions**:
  - Strict compliance with `TelemetryPacket` schema to prevent schema validation rejection on dashboard and ROS 2 ingest.
  - Shared I2C bus for MPU6050 and VL53L0X with zero address collision (0x68 vs 0x29).
  - Actuator safety: All flight motors remain strictly disarmed with 0 PWM on the bench.
- **Tests & Validation Performed**:
  - Validated generated JSON schema via Python test suite (`tests/test_firmware_telemetry_schema.py`) confirming 100% compliance with Pydantic model.
- **Current Status**: Complete and ready for physical hardware upload and bench testing.

