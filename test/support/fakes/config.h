// Fixture for the native tests; same shape as include/config.h.example.
// A real include/config.h takes precedence, and the tests do not depend on these values.
#include <IPAddress.h>

// WiFi settings
#define WLAN_SSID "TestWifi"
#define WLAN_PASSWD "testWifiPassword"

// Network settings
IPAddress gateway(192, 0, 2, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 0, 2, 1);

// MQTT broker settings
#define MQTT_BROKER "192.0.2.10"
#define MQTT_PORT 1883
#define MQTT_USER "user"
#define MQTT_PASSWORD "password"

// MQTT client settings
#define MQTT_CLIENT "esp-temperature-test"
#define MQTT_TOPIC "sensor/esp_temperature_test"

// Static IP of the ESPtemp
IPAddress ip(192, 0, 2, 2);
