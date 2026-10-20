#pragma once

#include <Arduino.h>

#include "IPAddress.h"

typedef enum
{
    WL_NO_SHIELD = 255,
    WL_IDLE_STATUS = 0,
    WL_NO_SSID_AVAIL = 1,
    WL_SCAN_COMPLETED = 2,
    WL_CONNECTED = 3,
    WL_CONNECT_FAILED = 4,
    WL_CONNECTION_LOST = 5,
    WL_WRONG_PASSWORD = 6,
    WL_DISCONNECTED = 7
} wl_status_t;

typedef enum
{
    WIFI_OFF = 0,
    WIFI_STA = 1,
    WIFI_AP = 2,
    WIFI_AP_STA = 3
} WiFiMode_t;

class ESP8266WiFiClass
{
public:
    bool mode(WiFiMode_t mode);
    bool forceSleepBegin(uint32_t sleepUs = 0);
    bool forceSleepWake();
    void persistent(bool persistent);
    bool config(IPAddress local, IPAddress dns, IPAddress gateway, IPAddress subnet);
    bool hostname(const char *name);
    wl_status_t begin(const char *ssid, const char *passphrase = nullptr, int32_t channel = 0,
                      const uint8_t *bssid = nullptr, bool connect = true);
    wl_status_t status();
    bool disconnect(bool wifiOff = false);
    int32_t channel();
    uint8_t *BSSID();
    IPAddress localIP();
    String macAddress();
};

extern ESP8266WiFiClass WiFi;

class WiFiClient
{
};
