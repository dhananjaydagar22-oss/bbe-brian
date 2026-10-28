#include "sensor_manager.h"

namespace BeeBrain {

SensorManager::SensorManager()
    : _mpu(Wire, Pins::MPU6050_I2C_ADDR),
      _tof(Wire, Pins::VL53L0X_I2C_ADDR),
      _flow(SPI, Pins::ADNS3080_CS_PIN, Pins::ADNS3080_RESET_PIN) {}

bool SensorManager::init() {
    // 1. Initialize I2C Bus for MPU6050 and VL53L0X
    Wire.begin(Pins::I2C_SDA_PIN, Pins::I2C_SCL_PIN, Pins::I2C_FREQ_HZ);
    delay(50);

    // 2. Initialize ADC for Battery Monitoring
    analogReadResolution(12);
    pinMode(Pins::BATT_ADC_PIN, INPUT);

    // 3. Initialize Sensors
    bool mpuOk  = _mpu.init();
    if (mpuOk) {
        _mpu.calibrateGyro(Config::MPU6050_CALIB_SAMPLES);
    }

    bool tofOk  = _tof.init();
    bool flowOk = _flow.init();

    return (mpuOk && tofOk && flowOk);
}

void SensorManager::update(SensorStatusData& status,
                           IMUData& imu,
                           ToFData& tof,
                           OpticalFlowData& flow,
                           BatteryData& battery) {
    // Update MPU6050 IMU
    bool mpuReadOk = _mpu.read(imu);
    status.imu = _mpu.isHealthy() && mpuReadOk;

    // Update VL53L0X ToF
    bool tofReadOk = _tof.read(tof);
    status.tof = _tof.isHealthy() && tofReadOk;

    // Update ADNS3080 Optical Flow
    bool flowReadOk = _flow.read(flow);
    status.optical_flow = _flow.isHealthy() && flowReadOk && flow.tracking_ok;

    // Barometer currently not fitted
    status.barometer = false;

    // Update Battery Status
    float battV = readBatteryVoltage();
    battery.voltage_v = battV;
    battery.state_of_charge_pct = estimateStateOfCharge(battV);
    battery.temperature_c = _mpu.getTemperatureC();
}

float SensorManager::readBatteryVoltage() {
    uint32_t adcSum = 0;
    constexpr int SAMPLES = 8;
    for (int i = 0; i < SAMPLES; ++i) {
        adcSum += analogRead(Pins::BATT_ADC_PIN);
    }
    float avgAdc = static_cast<float>(adcSum) / static_cast<float>(SAMPLES);
    float pinVoltage = (avgAdc / Pins::ADC_RESOLUTION) * Pins::ADC_REF_VOLTAGE;
    return pinVoltage * Pins::BATT_DIVIDER_RATIO;
}

float SensorManager::estimateStateOfCharge(float voltage) {
    if (voltage >= Config::BATT_VOLTAGE_MAX_V) return 100.0f;
    if (voltage <= Config::BATT_VOLTAGE_MIN_V) return 0.0f;

    // Linear approximation between 3.3V (0%) and 4.2V (100%) for 1S LiPo
    float soc = ((voltage - Config::BATT_VOLTAGE_MIN_V) /
                 (Config::BATT_VOLTAGE_MAX_V - Config::BATT_VOLTAGE_MIN_V)) * 100.0f;
    return constrain(soc, 0.0f, 100.0f);
}

} // namespace BeeBrain
