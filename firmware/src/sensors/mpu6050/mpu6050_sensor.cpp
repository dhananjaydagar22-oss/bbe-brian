#include "mpu6050_sensor.h"

namespace BeeBrain {

MPU6050Sensor::MPU6050Sensor(TwoWire& wire, uint8_t address)
    : _wire(wire), _address(address) {}

bool MPU6050Sensor::init() {
    _consecutiveFailures = 0;

    // Check WHO_AM_I register (expects 0x68 or 0x69)
    uint8_t whoAmI = 0;
    if (!readRegister(REG_WHO_AM_I, whoAmI)) {
        _isInitialized = false;
        return false;
    }

    if (whoAmI != 0x68 && whoAmI != 0x69) {
        _isInitialized = false;
        return false;
    }

    // Wake up MPU6050: clear SLEEP bit, select PLL with X axis gyroscope reference
    if (!writeRegister(REG_PWR_MGMT_1, 0x01)) {
        _isInitialized = false;
        return false;
    }
    delay(10);

    // Set sample rate divider: 1 kHz / (1 + 9) = 100 Hz output rate
    writeRegister(REG_SMPLRT_DIV, 0x09);

    // Set Digital Low Pass Filter (DLPF): Mode 3 (Accel ~44 Hz, Gyro ~42 Hz bandwidth)
    writeRegister(REG_CONFIG, 0x03);

    // Set Gyroscope Full Scale Range: +/- 500 deg/s (FS_SEL = 1 -> 0x08)
    writeRegister(REG_GYRO_CONFIG, 0x08);

    // Set Accelerometer Full Scale Range: +/- 4g (AFS_SEL = 1 -> 0x08)
    writeRegister(REG_ACCEL_CONFIG, 0x08);

    _isInitialized = true;
    return true;
}

bool MPU6050Sensor::calibrateGyro(uint16_t sampleCount) {
    if (!_isInitialized) {
        return false;
    }

    double sumGx = 0.0, sumGy = 0.0, sumGz = 0.0;
    double sumAx = 0.0, sumAy = 0.0, sumAz = 0.0;
    uint16_t validSamples = 0;

    for (uint16_t i = 0; i < sampleCount; ++i) {
        uint8_t buffer[14];
        if (readRegisters(REG_ACCEL_XOUT_H, buffer, 14)) {
            int16_t rawAx = (static_cast<int16_t>(buffer[0]) << 8) | buffer[1];
            int16_t rawAy = (static_cast<int16_t>(buffer[2]) << 8) | buffer[3];
            int16_t rawAz = (static_cast<int16_t>(buffer[4]) << 8) | buffer[5];

            int16_t rawGx = (static_cast<int16_t>(buffer[8])  << 8) | buffer[9];
            int16_t rawGy = (static_cast<int16_t>(buffer[10]) << 8) | buffer[11];
            int16_t rawGz = (static_cast<int16_t>(buffer[12]) << 8) | buffer[13];

            sumAx += static_cast<double>(rawAx) / ACCEL_SCALE_4G * Config::GRAVITY_MSS;
            sumAy += static_cast<double>(rawAy) / ACCEL_SCALE_4G * Config::GRAVITY_MSS;
            sumAz += static_cast<double>(rawAz) / ACCEL_SCALE_4G * Config::GRAVITY_MSS;

            sumGx += static_cast<double>(rawGx) / GYRO_SCALE_500DPS;
            sumGy += static_cast<double>(rawGy) / GYRO_SCALE_500DPS;
            sumGz += static_cast<double>(rawGz) / GYRO_SCALE_500DPS;

            validSamples++;
        }
        delay(5);
    }

    if (validSamples < (sampleCount / 2)) {
        return false;
    }

    _gyroBiasX = static_cast<float>(sumGx / validSamples);
    _gyroBiasY = static_cast<float>(sumGy / validSamples);
    _gyroBiasZ = static_cast<float>(sumGz / validSamples);

    _accelBiasX = static_cast<float>(sumAx / validSamples);
    _accelBiasY = static_cast<float>(sumAy / validSamples);
    // On bench level, +Z experiences +1g = +9.80665 m/s^2
    _accelBiasZ = static_cast<float>((sumAz / validSamples) - Config::GRAVITY_MSS);

    return true;
}

bool MPU6050Sensor::read(IMUData& imuData) {
    if (!_isInitialized) {
        _consecutiveFailures++;
        return false;
    }

    uint8_t buffer[14];
    if (!readRegisters(REG_ACCEL_XOUT_H, buffer, 14)) {
        _consecutiveFailures++;
        return false;
    }

    _consecutiveFailures = 0;

    int16_t rawAx   = (static_cast<int16_t>(buffer[0])  << 8) | buffer[1];
    int16_t rawAy   = (static_cast<int16_t>(buffer[2])  << 8) | buffer[3];
    int16_t rawAz   = (static_cast<int16_t>(buffer[4])  << 8) | buffer[5];
    int16_t rawTemp = (static_cast<int16_t>(buffer[6])  << 8) | buffer[7];
    int16_t rawGx   = (static_cast<int16_t>(buffer[8])  << 8) | buffer[9];
    int16_t rawGy   = (static_cast<int16_t>(buffer[10]) << 8) | buffer[11];
    int16_t rawGz   = (static_cast<int16_t>(buffer[12]) << 8) | buffer[13];

    // Compute temperature in Celsius: (TEMP_degC = (rawTemp / 340.0) + 36.53)
    _temperatureC = (static_cast<float>(rawTemp) / 340.0f) + 36.53f;

    // Convert Accelerometer to m/s^2
    imuData.accel_x_m_s2 = (static_cast<float>(rawAx) / ACCEL_SCALE_4G * Config::GRAVITY_MSS) - _accelBiasX;
    imuData.accel_y_m_s2 = (static_cast<float>(rawAy) / ACCEL_SCALE_4G * Config::GRAVITY_MSS) - _accelBiasY;
    imuData.accel_z_m_s2 = (static_cast<float>(rawAz) / ACCEL_SCALE_4G * Config::GRAVITY_MSS) - _accelBiasZ;

    // Convert Gyroscope to deg/s with bias subtraction
    imuData.gyro_x_dps = (static_cast<float>(rawGx) / GYRO_SCALE_500DPS) - _gyroBiasX;
    imuData.gyro_y_dps = (static_cast<float>(rawGy) / GYRO_SCALE_500DPS) - _gyroBiasY;
    imuData.gyro_z_dps = (static_cast<float>(rawGz) / GYRO_SCALE_500DPS) - _gyroBiasZ;

    return true;
}

bool MPU6050Sensor::writeRegister(uint8_t reg, uint8_t value) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.write(value);
    return (_wire.endTransmission() == 0);
}

bool MPU6050Sensor::readRegister(uint8_t reg, uint8_t& value) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) {
        return false;
    }

    if (_wire.requestFrom(_address, static_cast<uint8_t>(1)) != 1) {
        return false;
    }

    value = _wire.read();
    return true;
}

bool MPU6050Sensor::readRegisters(uint8_t startReg, uint8_t* buffer, size_t length) {
    _wire.beginTransmission(_address);
    _wire.write(startReg);
    if (_wire.endTransmission(false) != 0) {
        return false;
    }

    size_t received = _wire.requestFrom(_address, static_cast<uint8_t>(length));
    if (received != length) {
        return false;
    }

    for (size_t i = 0; i < length; ++i) {
        buffer[i] = _wire.read();
    }
    return true;
}

} // namespace BeeBrain
