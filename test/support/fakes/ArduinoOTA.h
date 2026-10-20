#pragma once

#include <functional>

#include <Arduino.h>

typedef enum
{
    OTA_AUTH_ERROR,
    OTA_BEGIN_ERROR,
    OTA_CONNECT_ERROR,
    OTA_RECEIVE_ERROR,
    OTA_END_ERROR
} ota_error_t;

#define U_FLASH 0
#define U_FS 100

class ArduinoOTAClass
{
public:
    void setHostname(const char *name);
    void begin(bool useMDNS = true);
    void handle();
    void onStart(std::function<void()> callback);
    void onEnd(std::function<void()> callback);
    void onError(std::function<void(ota_error_t)> callback);
    int getCommand();
};

extern ArduinoOTAClass ArduinoOTA;
