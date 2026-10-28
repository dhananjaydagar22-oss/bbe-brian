#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "include/pin_config.h"
#include "include/system_config.h"
#include "include/telemetry_types.h"

namespace BeeBrain {

class MPU6050Sensor {
public:
    MPU6050Sensor(TwoWire& wire = Wire, uint8_t address = Pins::MPU6050_I2C_ADDR);

    /**
     * @brief Initialize MPU6050, verify WHO_AM_I, set sample rates and ranges.
     * @return true if initialization succeeded, false otherwise.
     */
    bool init();

    /**
     * @brief Perform stationary gyro bias calibration.
     * @param sampleCount Number of samples to average (default from Config).
     * @return true if calibration completed successfully.
     */
    bool calibrateGyro(uint16_t sampleCount = Config::MPU6050_CALIB_SAMPLES);

    /**
     * @brief Read accelerometer and gyroscope measurements.
     * @param imuData Output struct to populate with calibrated SI values.
     * @return true if read was successful and valid.
     */
    bool read(IMUData& imuData);

    /**
     * @brief Return health status of the MPU6050.
     */
    bool isHealthy() const { return _isInitialized && (_consecutiveFailures == 0); }

    /**
     * @brief Get die temperature in degrees Celsius.
     */
    float getTemperatureC() const { return _temperatureC; }

private:
    TwoWire& _wire;
    uint8_t  _address;
    bool     _isInitialized       = false;
    uint32_t _consecutiveFailures = 0;

    // Zero-rate calibration bias offsets
    float _gyroBiasX = 0.0f;
    float _gyroBiasY = 0.0f;
    float _gyroBiasZ = 0.0f;

    float _accelBiasX = 0.0f;
    float _accelBiasY = 0.0f;
    float _accelBiasZ = 0.0f;

    float _temperatureC = 25.0f;

    // Register map definitions
    static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
    static constexpr uint8_t REG_CONFIG       = 0x1A;
    static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
    static constexpr uint8_t REG_TEMP_OUT_H   = 0x41;
    static constexpr uint8_t REG_GYRO_XOUT_H  = 0x43;
    static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t REG_WHO_AM_I     = 0x75;

    // Sensitivity scales
    static constexpr float ACCEL_SCALE_4G     = 8192.0f; // LSB per g for +/- 4g
    static constexpr float GYRO_SCALE_500DPS  = 65.5f;   // LSB per deg/s for +/- 500 dps

    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    bool readRegisters(uint8_t startReg, uint8_t* buffer, size_t length);
};

} // namespace BeeBrain
