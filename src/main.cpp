#include <Arduino.h>
#include <Adafruit_HTU21DF.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "config.h"

#ifndef ESPTEMP_VERSION
#define ESPTEMP_VERSION "v1.2" /**< The current version of the ESPtemp firmware*/
#endif

#ifndef OTA_ENABLE_GPIO
#define OTA_ENABLE_GPIO 14 /**< Connect this GPIO Pin to VCC to enter OTA wait loop after reset */
#endif

#ifndef HTU21DF_VCC_GPIO
#define HTU21DF_VCC_GPIO 15 /**< VCC for HTU21DF sensor */
#endif

#ifndef SLEEPTIME
#define SLEEPTIME 300e6 /**< Time in deepsleep */
#endif

#ifndef SLEEPTIME_EXTENDED
#define SLEEPTIME_EXTENDED 21600e6 /**< Time in deepsleep if an error occured while trying to connect to wifi (to avoid battery drainage) */
#endif

#ifndef DEBUG
#define DEBUG 0 /**< Set to 1 to debug over serial */
#endif

#ifndef OTA
#define OTA 1 /**< Set to 1 to enable OTA waiting loop */
#endif

#if DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__)
#define D_print(...) Serial.print(__VA_ARGS__)
#define D_write(...) Serial.write(__VA_ARGS__)
#define D_println(...) Serial.println(__VA_ARGS__)
#define D_printf(...) Serial.printf(__VA_ARGS__)
#define D_timestamp() Serial.printf("[%lu] ", millis())
#else
#define D_SerialBegin(bauds)
#define D_print(...)
#define D_write(...)
#define D_println(...)
#define D_printf(...)
#define D_timestamp()
#endif

unsigned long startMillis;

float humidity;
float temperature;
float batteryVoltage;
int8_t batteryPercentage;
uint_fast16_t adcValue;
char error[15];

uint8_t wifiRetries = 0;

bool isHTU21DFready = false;
bool isADCReady = false;
bool isWifiStarted = false;
bool isWifiReady = false;
bool isRegularWifi = false;
bool isMQTTStarted = false;
bool isMQTTReady = false;
bool isRtcValid = false;
bool isOTAEnabled = false;

uint8_t ledState = HIGH;

Adafruit_HTU21DF htu = Adafruit_HTU21DF();

WiFiClient wifiClient;
PubSubClient pubSubclient(wifiClient);

struct
{
    uint32_t crc32;    // 4 bytes
    uint8_t channel;   // 1 byte,   5 in total
    uint8_t ap_mac[6]; // 6 bytes, 11 in total
} rtcData;

// the CRC routine
uint32_t calculateCRC32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xffffffff;
    while (length--)
    {
        uint8_t c = *data++;
        for (uint32_t i = 0x80; i > 0; i >>= 1)
        {
            bool bit = crc & 0x80000000;
            if (c & i)
            {
                bit = !bit;
            }

            crc <<= 1;
            if (bit)
            {
                crc ^= 0x04c11db7;
            }
        }
    }

    return crc;
}

void setup()
{
    D_SerialBegin(115200);

    // internal LED off
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);

    // we disable WiFi, coming from DeepSleep, as we do not need it right away
    WiFi.mode(WIFI_OFF);
    WiFi.forceSleepBegin();

    D_println();
    D_timestamp();
    D_printf("CpuFreqMHz: %u MHz\n", ESP.getCpuFreqMHz());
    D_timestamp();
    D_print("MAC Address: ");
    D_println(WiFi.macAddress());

    // enable HTU21DF_VCC
    pinMode(HTU21DF_VCC_GPIO, OUTPUT);
    digitalWrite(HTU21DF_VCC_GPIO, HIGH);
    htu.begin();

    // Wait minimum sampling period
    startMillis = millis();

    if (ESP.rtcUserMemoryRead(0, (uint32_t *)&rtcData, sizeof(rtcData)))
    {
        // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
        uint32_t crc = calculateCRC32(((uint8_t *)&rtcData) + 4, sizeof(rtcData) - 4);
        if (crc == rtcData.crc32)
        {
            isRtcValid = true;
        }
    }

#if OTA
#if DEBUG
    ArduinoOTA.onStart([]()
                       {
        String __type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            __type = "sketch";
        else // U_SPIFFS
            __type = "filesystem";
        D_timestamp();
        D_println("(OTA) Update starting: " + __type); });

    ArduinoOTA.onEnd([]()
                     {
        D_timestamp();
        D_println("(OTA) Update finished "); });

    ArduinoOTA.onError([](ota_error_t error)
                       {
        D_print("(OTA) Error[");
        D_print(error);
        D_print("]: ");
        if (error == OTA_AUTH_ERROR)
            D_println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
            D_println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
            D_println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
            D_println("Receive Failed");
        else if (error == OTA_END_ERROR)
            D_println("End Failed"); });
#endif
    pinMode(OTA_ENABLE_GPIO, INPUT);
    isOTAEnabled = (digitalRead(OTA_ENABLE_GPIO) == HIGH);
    if (isOTAEnabled)
    {
        D_timestamp();
        D_println("GPIO 14 is high, OTA enabled");
    }
    else
    {
        D_timestamp();
        D_println("GPIO 14 is low, OTA not enabled");
    }
#endif
    D_timestamp();
    D_println("Setup finished");
}

void loop()
{

    /*
     * Step 1:
     * Read temperature and humidity from HTU21DF
     */
    if (!isHTU21DFready)
    {
        // Read sensors
        humidity = htu.readHumidity();
        temperature = htu.readTemperature();

        D_timestamp();
        D_println("HTU21DF Reading ready");
        D_timestamp();
        D_printf("Temperature: %.1f, Humidity: %.1f", temperature, humidity);
        D_println();

        isHTU21DFready = true;
        digitalWrite(HTU21DF_VCC_GPIO, LOW);
    }

    /*
     * Step 2:
     * Start WiFi connection
     */
    if (!isWifiStarted)
    {
        D_timestamp();
        D_println("Starting WiFi");

        WiFi.forceSleepBegin();
        delay(1);
        WiFi.forceSleepWake();
        delay(1);

        // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings unnecessarily in the flash memory.
        WiFi.persistent(false);

        // Bring up the WiFi connection
        WiFi.mode(WIFI_STA);
        WiFi.config(ip, dns, gateway, subnet);
        WiFi.hostname(MQTT_CLIENT);

        if (isRtcValid)
        {
            // The RTC data was good, make a quick connection
            D_timestamp();
            D_println("RTC OK, initiate quick connection");
            WiFi.begin(WLAN_SSID, WLAN_PASSWD, rtcData.channel, rtcData.ap_mac, true);
            isRegularWifi = false;
        }
        else
        {
            // The RTC data was not valid, so make a regular connection
            D_timestamp();
            D_println("RTC BAD, initiate regular connection");
            WiFi.begin(WLAN_SSID, WLAN_PASSWD);
            isRegularWifi = true;
        }

        isWifiStarted = true;
    }

    /*
     * Step 3:
     * Read battery voltage from ADC
     */
    if (!isADCReady)
    {
        adcValue = analogRead(A0);
        isADCReady = true;
        batteryVoltage = adcValue * 0.004;
        batteryPercentage = (int8_t)((adcValue - 863) / 1.6);

        if (batteryVoltage <= 3.3)
        {
            D_timestamp();
            D_println("Battery low. Going to long deep sleep. Good night.");
            ESP.deepSleep(0);
        }

        D_timestamp();
        D_println("ADC Reading ready");
        D_timestamp();
        D_printf("Battery voltage: %.1f V, Battery Percentage: %i", batteryVoltage, batteryPercentage);
        D_println();
    }

    /*
     * Step 4:
     * If WiFi is ready: Initialize MQTT client
     * OR
     * Start OTA waiting loop if OTA_ENABLE_GPIO is pulled HIGH
     */
    if (!isWifiReady)
    {
        if (WiFi.status() == WL_CONNECTED)
        {
            D_timestamp();
            D_print("WiFi connected (");
            D_print(WiFi.localIP());
            D_println(")");
            pubSubclient.setServer(MQTT_BROKER, MQTT_PORT);
            isWifiReady = true;

            if (isOTAEnabled)
            {
                D_timestamp();
                D_println("Starting OTA");
                ArduinoOTA.setHostname(MQTT_CLIENT);
                ArduinoOTA.begin();
            }
        }
        else if (((millis() - startMillis) > 5000) && (!isRegularWifi))
        {
            // Try regular connection
            D_timestamp();
            D_println("WiFi timeout, retrying regular connection");
            WiFi.disconnect();
            delay(5);
            WiFi.forceSleepBegin();
            delay(1);
            WiFi.forceSleepWake();
            delay(1);
            WiFi.begin(WLAN_SSID, WLAN_PASSWD);
            isRtcValid = false;
            isRegularWifi = true;
        }
        else if ((millis() - startMillis) > 20000)
        {
            // Wifi connection failed, go to extended sleep
            WiFi.disconnect(true);
            D_timestamp();
            D_println("Connection to WiFi failed, going to extended sleep");
            ESP.deepSleep(SLEEPTIME_EXTENDED, WAKE_RF_DISABLED);
        }
    }
    /*
     * Step 5:
     * Connect to MQTT broker
     */
    else if (!isMQTTStarted)
    {
        if (!pubSubclient.connected())
        {
            D_timestamp();
            D_println("Connecting to MQTT broker");
            pubSubclient.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASSWORD);
            delay(10);
        }
        else
        {
            D_timestamp();
            D_println("MQTT connected");
            isMQTTStarted = true;
        }
    }

    /*
     * Step 5:
     * If all readings are made and WiFi is ready, transmit MQTT data
     */
    if (isMQTTStarted && isWifiReady && isHTU21DFready && !isMQTTReady)
    {
        DynamicJsonDocument doc(1024);
        doc["temperature"] = (int)roundf(10 * temperature) / 10.0;
        doc["humidity"] = (int)roundf(10 * humidity) / 10.0;
        doc["batteryVoltage"] = (int)roundf(100 * batteryVoltage) / 100.0;
        doc["batteryPercentage"] = batteryPercentage;
        doc["firmwareVersion"] = ESPTEMP_VERSION;

        char json[128];
        serializeJson(doc, json);

        D_timestamp();
        D_print("MQTT Publish: ");
        D_print(MQTT_TOPIC);
        D_print(" ");
        D_println(json);

        pubSubclient.loop();
        pubSubclient.publish(MQTT_TOPIC, json);
        delay(10);

        isMQTTReady = true;
    }

    if (!isOTAEnabled)
    {
        /*
         * Step 6:
         * Save WiFi data to RTC and go to deep sleep again
         */
        if (isMQTTReady)
        {
            // Write current connection info to RTC
            if (!isRtcValid)
            {
                rtcData.channel = WiFi.channel();
                memcpy(rtcData.ap_mac, WiFi.BSSID(), 6); // Copy 6 bytes of BSSID (AP's MAC address)
                rtcData.crc32 = calculateCRC32(((uint8_t *)&rtcData) + 4, sizeof(rtcData) - 4);
                ESP.rtcUserMemoryWrite(0, (uint32_t *)&rtcData, sizeof(rtcData));
            }

            WiFi.disconnect(true);

            D_timestamp();
            D_println("DONE, going to sleep");
            ESP.deepSleep(SLEEPTIME, WAKE_RF_DISABLED);
        }
        else if ((millis() - startMillis) > 25000)
        {
            WiFi.disconnect(true);
            D_timestamp();
            D_println("TIMEOUT, going to sleep");
            ESP.deepSleep(SLEEPTIME, WAKE_RF_DISABLED);
        }
    }
    /*
     * OTA wait loop
     */
    else
    {
#if OTA
        // Blink LED
        if ((millis() % 1000) > 500)
        {
            ledState = HIGH;
        }
        else
        {
            ledState = LOW;
        }
        digitalWrite(LED_BUILTIN, ledState);
        ArduinoOTA.handle();
#endif
    }
}
