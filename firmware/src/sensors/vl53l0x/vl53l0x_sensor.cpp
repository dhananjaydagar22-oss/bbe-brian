#include "vl53l0x_sensor.h"

namespace BeeBrain {

VL53L0XSensor::VL53L0XSensor(TwoWire& wire, uint8_t address)
    : _wire(wire), _address(address) {}

bool VL53L0XSensor::init() {
    _consecutiveFailures = 0;

    // Check Model ID (expects 0xEE)
    uint8_t modelId = 0;
    if (!readReg(REG_IDENTIFICATION_MODEL_ID, modelId) || modelId != 0xEE) {
        _isInitialized = false;
        return false;
    }

    // Standard I2C standard mode voltage setup (2.8V IO)
    uint8_t extSupHv = 0;
    readReg(REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, extSupHv);
    writeReg(REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, extSupHv | 0x01);

    // Set I2C standard mode
    writeReg(0x88, 0x00);
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    readReg(0x91, _stopVariable);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);

    // Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks
    writeReg(REG_MSRC_CONFIG_CONTROL, 0x12);

    // Set return signal rate limit to 0.25 MCPS
    writeReg16Bit(REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32);

    writeReg(REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // Temperature / VHV and Phase calibration
    performSingleRefCalibration(0x40);
    performSingleRefCalibration(0x00);

    // Enable all sequence steps
    writeReg(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    _isInitialized = true;
    return true;
}

bool VL53L0XSensor::performSingleRefCalibration(uint8_t vhvInitByte) {
    writeReg(REG_SYSRANGE_START, 0x01 | vhvInitByte);
    uint32_t startMs = millis();
    uint8_t sysrange = 0;
    while ((millis() - startMs) < 100) {
        readReg(REG_SYSRANGE_START, sysrange);
        if ((sysrange & 0x01) == 0) {
            break;
        }
        delay(1);
    }
    writeReg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    writeReg(REG_SYSRANGE_START, 0x00);
    return true;
}

bool VL53L0XSensor::read(ToFData& tofData) {
    if (!_isInitialized) {
        tofData.measurement_valid = false;
        _consecutiveFailures++;
        return false;
    }

    // Trigger single-shot measurement
    writeReg(0x80, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, _stopVariable);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    writeReg(0x80, 0x00);
    writeReg(REG_SYSRANGE_START, 0x01);

    // Poll for measurement completion with timeout
    uint32_t startMs = millis();
    uint8_t sysrange = 0;
    bool ready = false;
    while ((millis() - startMs) < 35) { // 35 ms timeout
        readReg(REG_SYSRANGE_START, sysrange);
        if ((sysrange & 0x01) == 0) {
            ready = true;
            break;
        }
        delayMicroseconds(500);
    }

    if (!ready) {
        _consecutiveFailures++;
        tofData.measurement_valid = false;
        return false;
    }

    // Read range status and distance registers (0x14 through 0x1E)
    uint8_t rangeStatus = 0;
    readReg(REG_RESULT_RANGE_STATUS, rangeStatus);

    uint16_t rawDistanceMm = 0;
    readReg16Bit(REG_RESULT_RANGE_STATUS + 10, rawDistanceMm);

    // Clear interrupt
    writeReg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    // Range status: lower 4 bits (0 = range valid)
    uint8_t deviceRangeStatus = ((rangeStatus & 0x78) >> 3);

    // Compute signal rate (approximate MCPS from range status registers)
    uint16_t signalRateRaw = 0;
    readReg16Bit(REG_RESULT_RANGE_STATUS + 6, signalRateRaw);
    tofData.signal_rate = static_cast<float>(signalRateRaw) / 65536.0f * 8.0f;

    tofData.distance_mm = static_cast<float>(rawDistanceMm);

    // Measurement is valid if hardware status is valid (0 or 11) and within optical range
    if ((deviceRangeStatus == 0 || deviceRangeStatus == 11) &&
        rawDistanceMm >= Config::TOF_MIN_VALID_RANGE_MM &&
        rawDistanceMm <= Config::TOF_MAX_VALID_RANGE_MM) {
        tofData.measurement_valid = true;
        _consecutiveFailures = 0;
        return true;
    } else {
        tofData.measurement_valid = false;
        // Do not immediately count as hardware failure if surface was out of range
        return false;
    }
}

bool VL53L0XSensor::writeReg(uint8_t reg, uint8_t val) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.write(val);
    return (_wire.endTransmission() == 0);
}

bool VL53L0XSensor::writeReg16Bit(uint8_t reg, uint16_t val) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    _wire.write(static_cast<uint8_t>((val >> 8) & 0xFF));
    _wire.write(static_cast<uint8_t>(val & 0xFF));
    return (_wire.endTransmission() == 0);
}

bool VL53L0XSensor::readReg(uint8_t reg, uint8_t& val) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    if (_wire.requestFrom(_address, static_cast<uint8_t>(1)) != 1) return false;
    val = _wire.read();
    return true;
}

bool VL53L0XSensor::readReg16Bit(uint8_t reg, uint16_t& val) {
    _wire.beginTransmission(_address);
    _wire.write(reg);
    if (_wire.endTransmission(false) != 0) return false;
    if (_wire.requestFrom(_address, static_cast<uint8_t>(2)) != 2) return false;
    val = (static_cast<uint16_t>(_wire.read()) << 8);
    val |= _wire.read();
    return true;
}

} // namespace BeeBrain
