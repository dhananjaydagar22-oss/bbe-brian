#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "include/pin_config.h"
#include "include/system_config.h"
#include "include/telemetry_types.h"
#include "mpu6050/mpu6050_sensor.h"
#include "vl53l0x/vl53l0x_sensor.h"
#include "adns3080/adns3080_sensor.h"

namespace BeeBrain {

class SensorManager {
public:
    SensorManager();

    /**
     * @brief Initialize all hardware buses (I2C, SPI, ADC) and sensors.
     * @return true if all primary sensors initialized successfully.
     */
    bool init();

    /**
     * @brief Poll all active sensors and update telemetry data structures.
     * @param status Output sensor status flags.
     * @param imu Output IMU data.
     * @param tof Output ToF data.
     * @param flow Output optical flow data.
     * @param battery Output battery data.
     */
    void update(SensorStatusData& status,
                IMUData& imu,
                ToFData& tof,
                OpticalFlowData& flow,
                BatteryData& battery);

    MPU6050Sensor&  getMPU6050()  { return _mpu; }
    VL53L0XSensor&  getVL53L0X()  { return _tof; }
    ADNS3080Sensor& getADNS3080() { return _flow; }

private:
    MPU6050Sensor  _mpu;
    VL53L0XSensor  _tof;
    ADNS3080Sensor _flow;

    float readBatteryVoltage();
    float estimateStateOfCharge(float voltage);
};

} // namespace BeeBrain
