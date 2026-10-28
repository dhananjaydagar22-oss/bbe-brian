#pragma once

#include <Arduino.h>
#include <SPI.h>
#include "include/pin_config.h"
#include "include/system_config.h"
#include "include/telemetry_types.h"

namespace BeeBrain {

class ADNS3080Sensor {
public:
    ADNS3080Sensor(SPIClass& spi = SPI,
                   int csPin = Pins::ADNS3080_CS_PIN,
                   int resetPin = Pins::ADNS3080_RESET_PIN);

    /**
     * @brief Initialize SPI bus, reset ADNS3080, and verify Product ID.
     * @return true if initialization succeeded and Product ID is 0x17.
     */
    bool init();

    /**
     * @brief Read relative motion counts and surface quality.
     * @param flowData Output struct populated with delta_x, delta_y, quality, tracking_ok.
     * @return true if motion data was acquired successfully.
     */
    bool read(OpticalFlowData& flowData);

    /**
     * @brief Return health status.
     */
    bool isHealthy() const { return _isInitialized && (_consecutiveFailures < 5); }

private:
    SPIClass& _spi;
    int       _csPin;
    int       _resetPin;
    bool      _isInitialized       = false;
    uint32_t  _consecutiveFailures = 0;

    // ADNS3080 Register Map
    static constexpr uint8_t REG_PRODUCT_ID      = 0x00; // Expected: 0x17
    static constexpr uint8_t REG_REVISION_ID     = 0x01;
    static constexpr uint8_t REG_MOTION          = 0x02;
    static constexpr uint8_t REG_DELTA_X         = 0x03;
    static constexpr uint8_t REG_DELTA_Y         = 0x04;
    static constexpr uint8_t REG_SQUAL           = 0x05;
    static constexpr uint8_t REG_CONFIGURATION_BITS = 0x0A;
    static constexpr uint8_t REG_MOTION_BURST    = 0x50;

    static constexpr uint8_t EXPECTED_PRODUCT_ID = 0x17;

    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    void resetHardware();
};

} // namespace BeeBrain
