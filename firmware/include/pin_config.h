#pragma once

#include <Arduino.h>

/**
 * @file pin_config.h
 * @brief Hardware pin mapping for Seeed Studio XIAO ESP32-S3.
 *
 * Details the physical pin connections between the XIAO ESP32-S3,
 * the I2C sensors (MPU6050, VL53L0X), SPI sensor (ADNS3080),
 * battery monitor, bench SG90 servo, and bench motor PWM channels.
 */

namespace BeeBrain {
namespace Pins {

    // =========================================================================
    // I2C Bus Configuration (Shared by MPU6050 and VL53L0X)
    // =========================================================================
    constexpr int I2C_SDA_PIN          = 5;   // XIAO D4 / GPIO 5
    constexpr int I2C_SCL_PIN          = 6;   // XIAO D5 / GPIO 6
    constexpr uint32_t I2C_FREQ_HZ     = 400000; // 400 kHz Fast-mode

    // I2C Device Addresses
    constexpr uint8_t MPU6050_I2C_ADDR = 0x68; // AD0 tied to GND
    constexpr uint8_t VL53L0X_I2C_ADDR = 0x29; // Factory default

    // =========================================================================
    // SPI Bus Configuration (ADNS3080 Optical Flow Sensor)
    // =========================================================================
    constexpr int SPI_SCK_PIN          = 7;   // XIAO D8  / GPIO 7
    constexpr int SPI_MISO_PIN         = 8;   // XIAO D9  / GPIO 8
    constexpr int SPI_MOSI_PIN         = 9;   // XIAO D10 / GPIO 9
    constexpr int ADNS3080_CS_PIN      = 4;   // XIAO D3  / GPIO 4
    constexpr int ADNS3080_RESET_PIN   = 3;   // XIAO D2  / GPIO 3
    constexpr uint32_t SPI_FREQ_HZ     = 2000000; // 2 MHz SPI clock

    // =========================================================================
    // Battery Voltage Sensing
    // =========================================================================
    constexpr int BATT_ADC_PIN         = 1;   // XIAO A0 / D0 / GPIO 1 (ADC1_CH0)
    constexpr float BATT_DIVIDER_RATIO = 2.0f;// 1S LiPo resistor divider (e.g. 10k/10k)
    constexpr float ADC_REF_VOLTAGE    = 3.3f;// ESP32-S3 ADC reference
    constexpr float ADC_RESOLUTION     = 4095.0f; // 12-bit ADC

    // =========================================================================
    // Actuators & Bench Testing Pins
    // =========================================================================
    // SG90 Servo for bench-mounted test rig / pan-tilt scanner (NOT flight motor)
    constexpr int SERVO_BENCH_PIN      = 44;  // XIAO D7 / GPIO 44

    // 4x Coreless Motor PWM pins (bench scaffold, disarmed by default)
    constexpr int MOTOR_FL_PIN         = 41;  // GPIO 41
    constexpr int MOTOR_FR_PIN         = 42;  // GPIO 42
    constexpr int MOTOR_RR_PIN         = 38;  // GPIO 38
    constexpr int MOTOR_RL_PIN         = 39;  // GPIO 39

    // =========================================================================
    // Debug & Status
    // =========================================================================
    constexpr int DEBUG_SERIAL_TX_PIN  = 43;  // XIAO D6 / GPIO 43 (U0TXD / USB CDC)
    constexpr int STATUS_LED_PIN       = 21;  // User LED (if fitted on carrier)

} // namespace Pins
} // namespace BeeBrain
