#pragma once

#include <Arduino.h>
#include <ESP8266WiFi.h>

class PubSubClient
{
public:
    explicit PubSubClient(WiFiClient &) {}
    PubSubClient &setServer(const char *domain, uint16_t port);
    bool connect(const char *id, const char *user, const char *pass);
    bool connected();
    bool loop();
    bool publish(const char *topic, const char *payload);
};
