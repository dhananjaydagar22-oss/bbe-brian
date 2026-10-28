#include <Arduino.h>
#include "include/pin_config.h"
#include "include/system_config.h"
#include "include/telemetry_types.h"
#include "sensors/sensor_manager.h"
#include "control/state_estimation/state_estimator.h"
#include "control/pid/pid.h"
#include "control/motor_mixer/motor_mixer.h"
#include "bio/reichardt/reichardt_detector.h"
#include "bio/lgmd/lgmd_detector.h"
#include "safety/failsafe/failsafe.h"
#include "communication/telemetry/telemetry.h"

using namespace BeeBrain;

// =============================================================================
// Global Subsystem Singletons
// =============================================================================
static SensorManager      g_sensorManager;
static StateEstimator     g_stateEstimator;
static MotorMixer         g_motorMixer;
static ReichardtDetector  g_reichardtX(Config::REICHARDT_TAU_S);
static ReichardtDetector  g_reichardtY(Config::REICHARDT_TAU_S);
static LGMDDetector       g_lgmdDetector(Config::LGMD_COLLISION_VEL_THRESH);
static FailsafeManager    g_failsafe;
static TelemetryPublisher g_telemetryPublisher;

// =============================================================================
// Thread-Safe Shared State
// =============================================================================
static FullTelemetrySnapshot g_telemetrySnapshot;
static portMUX_TYPE          g_stateMutex = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// Core 1: Real-Time Fast Sensor Acquisition & Estimation Task (100 Hz / 10 ms)
// =============================================================================
void SensorTask(void* pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(Config::SENSOR_TASK_PERIOD_MS);

    uint32_t lastLoopTimeUs = micros();
    uint32_t servoStepCounter = 0;

    for (;;) {
        vTaskDelayUntil(&lastWakeTime, periodTicks);

        uint32_t nowUs = micros();
        float dt = static_cast<float>(nowUs - lastLoopTimeUs) / 1000000.0f;
        lastLoopTimeUs = nowUs;
        if (dt <= 0.0001f || dt > 0.05f) {
            dt = Config::SENSOR_TASK_DT_S;
        }

        uint32_t processStartUs = micros();

        // Local working structures
        SensorStatusData status;
        IMUData          imu;
        ToFData          tof;
        OpticalFlowData  flow;
        BatteryData      battery;
        FlightData       flight;
        MotorData        motors;
        SystemData       system;

        float altCm  = 0.0f;
        float driftX = 0.0f;
        float driftY = 0.0f;

        // 1. Read All Physical Sensors
        g_sensorManager.update(status, imu, tof, flow, battery);

        // 2. Run Bio-Inspired Elementary Motion Detector on Optical Flow
        if (flow.tracking_ok) {
            flow.velocity_x_cm_s = g_reichardtX.process(flow.delta_x, dt);
            flow.velocity_y_cm_s = g_reichardtY.process(flow.delta_y, dt);
        }

        // 3. Run State Estimator (Attitude, Altitude, Metric Drift)
        g_stateEstimator.update(imu, tof, flow, flight, altCm, driftX, driftY, dt);

        // 4. Run LGMD Looming Collision Detection
        bool loomingThreat = g_lgmdDetector.update(altCm, dt);

        // 5. Update Failsafe Safety State Machine
        g_failsafe.update(status, battery, loomingThreat, flight, system);

        // 6. Bench Actuator Update (SG90 Servo Sweep every 20 ms / 2 cycles)
        if (++servoStepCounter >= 2) {
            servoStepCounter = 0;
            g_motorMixer.stepBenchServoSweep();
        }

        // 7. Update Motor Outputs (Strictly 0 unless armed bench mode)
        g_motorMixer.updateMotors(motors, g_failsafe.isArmed());

        // Measure processing duration
        float processMs = static_cast<float>(micros() - processStartUs) / 1000.0f;
        system.processing_time_ms = processMs;
        system.uptime_s = static_cast<float>(millis()) / 1000.0f;
        system.loop_rate_hz = static_cast<float>(Config::SENSOR_TASK_FREQ_HZ);

        // Copy to shared state with spinlock
        taskENTER_CRITICAL(&g_stateMutex);
        g_telemetrySnapshot.altitude_cm  = altCm;
        g_telemetrySnapshot.drift_x      = driftX;
        g_telemetrySnapshot.drift_y      = driftY;
        g_telemetrySnapshot.battery_v    = battery.voltage_v;
        g_telemetrySnapshot.motor_pwm[0] = motors.pwm[0];
        g_telemetrySnapshot.motor_pwm[1] = motors.pwm[1];
        g_telemetrySnapshot.motor_pwm[2] = motors.pwm[2];
        g_telemetrySnapshot.motor_pwm[3] = motors.pwm[3];
        g_telemetrySnapshot.link_ok      = true;
        g_telemetrySnapshot.sensor_status= status;

        g_telemetrySnapshot.tof          = tof;
        g_telemetrySnapshot.optical_flow = flow;
        g_telemetrySnapshot.imu          = imu;
        g_telemetrySnapshot.flight       = flight;
        g_telemetrySnapshot.battery      = battery;
        g_telemetrySnapshot.motors       = motors;
        g_telemetrySnapshot.system       = system;
        taskEXIT_CRITICAL(&g_stateMutex);
    }
}

// =============================================================================
// Core 0: Telemetry Serialization & Wi-Fi UDP Streaming Task (50 Hz / 20 ms)
// =============================================================================
void TelemetryTask(void* pvParameters) {
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(Config::TELEMETRY_TASK_PERIOD_MS);

    for (;;) {
        vTaskDelayUntil(&lastWakeTime, periodTicks);

        FullTelemetrySnapshot snapshotCopy;

        // Atomically copy latest snapshot
        taskENTER_CRITICAL(&g_stateMutex);
        snapshotCopy = g_telemetrySnapshot;
        taskEXIT_CRITICAL(&g_stateMutex);

        // Transmit via Serial and Wi-Fi UDP
        g_telemetryPublisher.publish(snapshotCopy);
    }
}

// =============================================================================
// Arduino Setup & Initialization
// =============================================================================
void setup() {
    Serial.begin(Config::SERIAL_BAUD_RATE);
    delay(1000);

    Serial.println();
    Serial.println("==================================================");
    Serial.println("       BEE-BRAIN EMBEDDED SENSOR FIRMWARE         ");
    Serial.println("       Target: Seeed Studio XIAO ESP32-S3         ");
    Serial.println("       Stage 1: Reliable Data Acquisition         ");
    Serial.println("==================================================");

    // 1. Initialize Actuators (Bench Servo & Safe Motor Lock)
    Serial.print("[INIT] Actuator Subsystem (SG90 Servo & Motor PWM)... ");
    if (g_motorMixer.init()) {
        Serial.println("OK (DISARMED / BENCH MODE)");
    } else {
        Serial.println("FAILED");
    }

    // 2. Initialize Physical Sensors (MPU6050, VL53L0X, ADNS3080)
    Serial.print("[INIT] Initializing I2C, SPI, and Physical Sensors... ");
    bool sensorsOk = g_sensorManager.init();
    if (sensorsOk) {
        Serial.println("ALL SENSORS DETECTED");
    } else {
        Serial.println("PARTIAL / SENSOR RETRY ACTIVE");
    }

    Serial.print("  - MPU6050 (6-DOF IMU):       ");
    Serial.println(g_sensorManager.getMPU6050().isHealthy() ? "READY (0x68)" : "FAIL");

    Serial.print("  - VL53L0X (Downward ToF):    ");
    Serial.println(g_sensorManager.getVL53L0X().isHealthy() ? "READY (0x29)" : "FAIL");

    Serial.print("  - ADNS3080 (Optical Flow):   ");
    Serial.println(g_sensorManager.getADNS3080().isHealthy() ? "READY (SPI)" : "FAIL");

    // 3. Initialize Wi-Fi & UDP Telemetry Stream
    Serial.print("[INIT] Starting Wi-Fi Station & UDP Socket (Port 8888)... ");
    g_telemetryPublisher.init(Config::DEFAULT_WIFI_SSID,
                              Config::DEFAULT_WIFI_PASS,
                              Config::PI_FORWARDER_IP,
                              Config::UDP_TELEMETRY_PORT);
    Serial.println("ASYNC STARTED");

    // 4. Launch FreeRTOS Dual-Core Tasks
    Serial.println("[INIT] Launching Dual-Core FreeRTOS Tasks...");

    // Core 1: Sensor & Estimation Loop (100 Hz, Priority 5)
    xTaskCreatePinnedToCore(
        SensorTask,
        "SensorTask",
        Config::STACK_SENSOR_TASK,
        nullptr,
        Config::PRIO_SENSOR_TASK,
        nullptr,
        Config::CORE_REALTIME_SENSORS
    );

    // Core 0: Telemetry & Network Loop (50 Hz, Priority 2)
    xTaskCreatePinnedToCore(
        TelemetryTask,
        "TelemetryTask",
        Config::STACK_TELEMETRY_TASK,
        nullptr,
        Config::PRIO_TELEMETRY_TASK,
        nullptr,
        Config::CORE_COMMUNICATIONS
    );

    Serial.println("[SYSTEM] System Operational. Streaming Telemetry.");
    Serial.println("==================================================");
}

void loop() {
    // FreeRTOS handles all tasks; keep Arduino loop low-priority idle.
    vTaskDelay(pdMS_TO_TICKS(1000));
}