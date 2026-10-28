#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "include/pin_config.h"
#include "include/system_config.h"
#include "include/telemetry_types.h"

namespace BeeBrain {

class VL53L0XSensor {
public:
    VL53L0XSensor(TwoWire& wire = Wire, uint8_t address = Pins::VL53L0X_I2C_ADDR);

    /**
     * @brief Initialize VL53L0X ToF rangefinder, verify identification and configure registers.
     * @return true if initialization succeeded, false otherwise.
     */
    bool init();

    /**
     * @brief Perform distance measurement.
     * @param tofData Output struct populated with distance_mm, signal_rate, and validity.
     * @return true if valid measurement acquired.
     */
    bool read(ToFData& tofData);

    /**
     * @brief Return health status.
     */
    bool isHealthy() const { return _isInitialized && (_consecutiveFailures < 5); }

private:
    TwoWire& _wire;
    uint8_t  _address;
    bool     _isInitialized       = false;
    uint32_t _consecutiveFailures = 0;
    uint8_t  _stopVariable        = 0;

    // Registers
    static constexpr uint8_t REG_IDENTIFICATION_MODEL_ID = 0xC0;
    static constexpr uint8_t REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89;
    static constexpr uint8_t REG_MSRC_CONFIG_CONTROL    = 0x60;
    static constexpr uint8_t REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44;
    static constexpr uint8_t REG_SYSTEM_SEQUENCE_CONFIG = 0x01;
    static constexpr uint8_t REG_DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;
    static constexpr uint8_t REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
    static constexpr uint8_t REG_GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
    static constexpr uint8_t REG_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
    static constexpr uint8_t REG_GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
    static constexpr uint8_t REG_SYSTEM_INTERRUPT_CLEAR = 0x0B;
    static constexpr uint8_t REG_RESULT_INTERRUPT_STATUS = 0x13;
    static constexpr uint8_t REG_RESULT_RANGE_STATUS = 0x14;
    static constexpr uint8_t REG_SYSRANGE_START = 0x00;

    bool writeReg(uint8_t reg, uint8_t val);
    bool writeReg16Bit(uint8_t reg, uint16_t val);
    bool readReg(uint8_t reg, uint8_t& val);
    bool readReg16Bit(uint8_t reg, uint16_t& val);
    bool performSingleRefCalibration(uint8_t vhvInitByte);
};

} // namespace BeeBrain
