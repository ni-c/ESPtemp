#include "fake.h"

#include <Adafruit_HTU21DF.h>
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

namespace fake
{
    World world;
    Boot boot;
    uint8_t rtc[512];

    void reset()
    {
        world = World();
        boot = Boot();
        memset(rtc, 0, sizeof(rtc));
    }

    void newBoot()
    {
        boot = Boot();
    }
}

using fake::boot;
using fake::world;

EspClass ESP;
ESP8266WiFiClass WiFi;
ArduinoOTAClass ArduinoOTA;

// Arduino core

unsigned long millis() { return boot.now; }

void delay(unsigned long ms) { boot.now += ms; }

void pinMode(uint8_t pin, uint8_t mode) { boot.pinModes[pin] = mode; }

void digitalWrite(uint8_t pin, uint8_t value)
{
    boot.pinLevels[pin] = value;
    if (value == HIGH)
    {
        boot.pinHighWrites[pin]++;
    }
    else
    {
        boot.pinLowWrites[pin]++;
    }
}

int digitalRead(uint8_t pin) { return (world.highInputs >> pin) & 1 ? HIGH : LOW; }

int analogRead(uint8_t) { return world.adc; }

void EspClass::deepSleep(uint64_t timeUs, RFMode mode)
{
    throw fake::DeepSleep{timeUs, mode};
}

bool EspClass::rtcUserMemoryRead(uint32_t offset, uint32_t *data, size_t size)
{
    if (world.rtcReadFails || offset * 4 + size > sizeof(fake::rtc))
    {
        return false;
    }
    memcpy(data, fake::rtc + offset * 4, size);
    return true;
}

bool EspClass::rtcUserMemoryWrite(uint32_t offset, uint32_t *data, size_t size)
{
    if (offset * 4 + size > sizeof(fake::rtc))
    {
        return false;
    }
    memcpy(fake::rtc + offset * 4, data, size);
    boot.rtcWrites++;
    return true;
}

uint8_t EspClass::getCpuFreqMHz() { return 160; }

// ESP8266WiFi

bool ESP8266WiFiClass::mode(WiFiMode_t mode)
{
    boot.wifiMode = mode;
    return true;
}

bool ESP8266WiFiClass::forceSleepBegin(uint32_t) { return true; }

bool ESP8266WiFiClass::forceSleepWake() { return true; }

void ESP8266WiFiClass::persistent(bool persistent) { boot.persistent = persistent; }

bool ESP8266WiFiClass::config(IPAddress local, IPAddress, IPAddress, IPAddress)
{
    boot.ip = local.value;
    return true;
}

bool ESP8266WiFiClass::hostname(const char *name)
{
    boot.hostname = name;
    return true;
}

wl_status_t ESP8266WiFiClass::begin(const char *ssid, const char *, int32_t channel, const uint8_t *bssid, bool)
{
    boot.ssid = ssid;
    boot.wifiBegun = true;
    boot.wifiBegunAt = boot.now;
    boot.wifiBegins++;
    boot.wifiQuick = bssid != nullptr;
    if (boot.wifiQuick)
    {
        boot.quickBegins++;
        boot.quickChannel = channel;
        memcpy(boot.quickBssid, bssid, 6);
    }
    return status();
}

wl_status_t ESP8266WiFiClass::status()
{
    if (!boot.wifiBegun || boot.wifiMode != WIFI_STA)
    {
        return WL_DISCONNECTED;
    }
    bool reachable = !boot.wifiQuick || world.quickConnectWorks;
    if (reachable && world.wifiConnectAfterMs >= 0 &&
        boot.now - boot.wifiBegunAt >= (unsigned long)world.wifiConnectAfterMs)
    {
        return WL_CONNECTED;
    }
    return (wl_status_t)world.wifiFailStatus;
}

bool ESP8266WiFiClass::disconnect(bool)
{
    boot.wifiBegun = false;
    boot.disconnects++;
    return true;
}

int32_t ESP8266WiFiClass::channel() { return world.channel; }

uint8_t *ESP8266WiFiClass::BSSID() { return world.bssid; }

IPAddress ESP8266WiFiClass::localIP() { return IPAddress(192, 0, 2, 2); }

String ESP8266WiFiClass::macAddress() { return "02:00:00:00:00:01"; }

// PubSubClient

PubSubClient &PubSubClient::setServer(const char *domain, uint16_t port)
{
    boot.mqttServer = domain;
    boot.mqttPort = port;
    return *this;
}

bool PubSubClient::connect(const char *id, const char *, const char *)
{
    boot.clientId = id;
    boot.mqttConnects++;
    boot.now += world.mqttConnectCostMs;
    boot.mqttConnected = world.mqttReachable && WiFi.status() == WL_CONNECTED;
    return boot.mqttConnected;
}

bool PubSubClient::connected() { return boot.mqttConnected; }

bool PubSubClient::loop() { return boot.mqttConnected; }

bool PubSubClient::publish(const char *topic, const char *payload)
{
    boot.topic = topic;
    boot.payload = payload;
    boot.publishes++;
    return boot.mqttConnected;
}

// Adafruit_HTU21DF

bool Adafruit_HTU21DF::begin()
{
    boot.htuBegins++;
    return true;
}

float Adafruit_HTU21DF::readTemperature() { return world.temperature; }

float Adafruit_HTU21DF::readHumidity() { return world.humidity; }

// ArduinoOTA

void ArduinoOTAClass::setHostname(const char *name) { boot.otaHostname = name; }

void ArduinoOTAClass::begin(bool) { boot.otaBegun = true; }

void ArduinoOTAClass::handle() { boot.otaHandles++; }

void ArduinoOTAClass::onStart(std::function<void()>) {}

void ArduinoOTAClass::onEnd(std::function<void()>) {}

void ArduinoOTAClass::onError(std::function<void(ota_error_t)>) {}

int ArduinoOTAClass::getCommand() { return U_FLASH; }
