#pragma once

// Test doubles for the ESP8266 Arduino core and the libraries ESPtemp uses,
// so that src/main.cpp runs unchanged on the build machine.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>

namespace fake
{
    // ESP.deepSleep() never returns on the chip; here it unwinds the boot.
    struct DeepSleep
    {
        uint64_t us;
        int mode;
    };

    // How the world behaves. Survives deep sleep.
    struct World
    {
        long wifiConnectAfterMs = 100; // after WiFi.begin(); negative: never
        bool quickConnectWorks = true; // with the saved channel and BSSID
        int wifiFailStatus = 7;        // what WiFi.status() says until connected (WL_DISCONNECTED)
        uint8_t channel = 6;
        uint8_t bssid[6] = {0x02, 0x11, 0x22, 0x33, 0x44, 0x55};

        bool mqttReachable = true;
        unsigned long mqttConnectCostMs = 50;

        float temperature = 21.46f;
        float humidity = 45.34f;
        uint16_t adc = 1000;
        uint32_t highInputs = 0; // bit n: digitalRead(n) is HIGH
        bool rtcReadFails = false;
    };

    // What one boot did. Starts empty after every wake.
    struct Boot
    {
        unsigned long now = 0;

        int pinModes[32] = {};
        int pinLevels[32] = {};
        int pinHighWrites[32] = {};
        int pinLowWrites[32] = {};
        int rtcWrites = 0;

        int wifiMode = -1;
        bool wifiBegun = false;
        bool wifiQuick = false;
        unsigned long wifiBegunAt = 0;
        int wifiBegins = 0;
        int quickBegins = 0;
        int quickChannel = -1;
        uint8_t quickBssid[6] = {};
        std::string ssid;
        std::string hostname;
        bool persistent = true;
        uint32_t ip = 0;
        int disconnects = 0;

        std::string mqttServer;
        uint16_t mqttPort = 0;
        std::string clientId;
        int mqttConnects = 0;
        bool mqttConnected = false;
        std::string topic;
        std::string payload;
        int publishes = 0;

        std::string otaHostname;
        bool otaBegun = false;
        long otaHandles = 0;

        int htuBegins = 0;
    };

    extern World world;
    extern Boot boot;
    extern uint8_t rtc[512];

    // Power on: a new world, empty RTC memory.
    void reset();
    // Wake from deep sleep: RTC memory and the world stay.
    void newBoot();
}
