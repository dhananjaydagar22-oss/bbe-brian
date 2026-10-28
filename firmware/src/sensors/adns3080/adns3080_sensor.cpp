#include "adns3080_sensor.h"

namespace BeeBrain {

ADNS3080Sensor::ADNS3080Sensor(SPIClass& spi, int csPin, int resetPin)
    : _spi(spi), _csPin(csPin), _resetPin(resetPin) {}

void ADNS3080Sensor::resetHardware() {
    pinMode(_resetPin, OUTPUT);
    digitalWrite(_resetPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(_resetPin, LOW);
    delayMicroseconds(500);
}

bool ADNS3080Sensor::init() {
    _consecutiveFailures = 0;

    pinMode(_csPin, OUTPUT);
    digitalWrite(_csPin, HIGH);

    // Initialize SPI bus pins for ESP32-S3
    _spi.begin(Pins::SPI_SCK_PIN, Pins::SPI_MISO_PIN, Pins::SPI_MOSI_PIN, Pins::ADNS3080_CS_PIN);

    // Perform hardware reset
    resetHardware();

    // Verify Product ID
    uint8_t prodId = readRegister(REG_PRODUCT_ID);
    if (prodId != EXPECTED_PRODUCT_ID) {
        _isInitialized = false;
        return false;
    }

    // Set configuration bits: 1600 counts per inch (CPI), LED continuous mode
    writeRegister(REG_CONFIGURATION_BITS, 0x10);

    // Clear motion register
    writeRegister(REG_MOTION, 0x00);

    _isInitialized = true;
    return true;
}

bool ADNS3080Sensor::read(OpticalFlowData& flowData) {
    if (!_isInitialized) {
        flowData.tracking_ok = false;
        _consecutiveFailures++;
        return false;
    }

    // Read Motion register (bit 7: motion occurred, bit 4: fault/overflow)
    uint8_t motion = readRegister(REG_MOTION);
    uint8_t rawDx  = readRegister(REG_DELTA_X);
    uint8_t rawDy  = readRegister(REG_DELTA_Y);
    uint8_t squal  = readRegister(REG_SQUAL);

    _consecutiveFailures = 0;

    // Convert raw 8-bit registers to signed counts
    int8_t dx = static_cast<int8_t>(rawDx);
    int8_t dy = static_cast<int8_t>(rawDy);

    flowData.delta_x = static_cast<float>(dx);
    flowData.delta_y = static_cast<float>(dy);

    // Normalize surface quality to 0.0 - 1.0 range
    flowData.quality = static_cast<float>(squal) / 255.0f;

    // Tracking is considered OK if SQUAL exceeds threshold and no sensor overflow
    bool overflow = (motion & 0x10) != 0;
    flowData.tracking_ok = (squal >= Config::FLOW_MIN_SQUAL) && !overflow;

    return true;
}

uint8_t ADNS3080Sensor::readRegister(uint8_t reg) {
    digitalWrite(_csPin, LOW);
    _spi.beginTransaction(SPISettings(Pins::SPI_FREQ_HZ, MSBFIRST, SPI_MODE3));

    // Send register address with MSB = 0 for read
    _spi.transfer(reg & 0x7F);
    delayMicroseconds(75); // Turnaround delay tSRAD per datasheet

    uint8_t value = _spi.transfer(0x00);

    _spi.endTransaction();
    digitalWrite(_csPin, HIGH);
    delayMicroseconds(1); // tSCLK-CS per datasheet

    return value;
}

void ADNS3080Sensor::writeRegister(uint8_t reg, uint8_t value) {
    digitalWrite(_csPin, LOW);
    _spi.beginTransaction(SPISettings(Pins::SPI_FREQ_HZ, MSBFIRST, SPI_MODE3));

    // Send register address with MSB = 1 for write
    _spi.transfer(reg | 0x80);
    delayMicroseconds(20);

    _spi.transfer(value);

    _spi.endTransaction();
    digitalWrite(_csPin, HIGH);
    delayMicroseconds(20); // tSWW write-to-write delay
}

} // namespace BeeBrain
