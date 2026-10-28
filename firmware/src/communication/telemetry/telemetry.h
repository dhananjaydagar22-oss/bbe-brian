#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "include/pin_config.h"
#include "include/system_config.h"
#include "include/telemetry_types.h"

namespace BeeBrain {

class TelemetryPublisher {
public:
    TelemetryPublisher();

    /**
     * @brief Initialize Wi-Fi connection and UDP socket.
     * @param ssid Wi-Fi network SSID (Raspberry Pi AP or local router).
     * @param password Wi-Fi password.
     * @param destinationIp Destination IP for UDP packets (Raspberry Pi 3 B+).
     * @param port Destination UDP port (default 8888).
     * @return true if Wi-Fi initialization started.
     */
    bool init(const char* ssid = Config::DEFAULT_WIFI_SSID,
              const char* password = Config::DEFAULT_WIFI_PASS,
              const char* destinationIp = Config::PI_FORWARDER_IP,
              uint16_t port = Config::UDP_TELEMETRY_PORT);

    /**
     * @brief Serialize FullTelemetrySnapshot into strict JSON conforming to TelemetryPacket.
     * @param snapshot Telemetry data snapshot.
     * @param outputBuffer Destination char array.
     * @param maxLen Maximum buffer length.
     * @return Number of characters written.
     */
    size_t serializeJSON(const FullTelemetrySnapshot& snapshot,
                         char* outputBuffer,
                         size_t maxLen);

    /**
     * @brief Publish telemetry snapshot via both USB Serial and Wi-Fi UDP.
     * @param snapshot Telemetry data snapshot.
     * @return true if published successfully.
     */
    bool publish(const FullTelemetrySnapshot& snapshot);

    /**
     * @brief Check Wi-Fi link status and reconnect if disconnected.
     */
    void maintainConnection();

    bool isWifiConnected() const { return (WiFi.status() == WL_CONNECTED); }
    float getWifiRssiDbm() const { return static_cast<float>(WiFi.RSSI()); }
    uint32_t getSequenceNumber() const { return _sequenceNumber; }

private:
    WiFiUDP     _udp;
    const char* _destIp;
    uint16_t    _port;
    uint32_t    _sequenceNumber = 0;
    char        _jsonBuffer[1536];

    uint32_t    _lastWifiCheckMs = 0;
    uint32_t    _packetsSent    = 0;
    uint32_t    _packetsDropped = 0;
};

} // namespace BeeBrain
