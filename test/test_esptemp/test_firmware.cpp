#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <unity.h>

#include <vector>

#include "harness.h"

extern IPAddress ip; // config.h, compiled into src/main.cpp

static const uint64_t PERIOD = 300e6; // SLEEPTIME
static const uint8_t OTA_PIN = 14;
static const uint8_t SENSOR_VCC = 15;

static StaticJsonDocument<512> published()
{
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, fake::boot.payload);
    TEST_ASSERT_FALSE_MESSAGE(error, fake::boot.payload.c_str());
    return doc;
}

static void assertSleptOnePeriodWithoutRadio(const Wake &result)
{
    TEST_ASSERT_TRUE(result.slept);
    TEST_ASSERT_EQUAL_UINT64(PERIOD, result.us);
    TEST_ASSERT_EQUAL_INT(WAKE_RF_DISABLED, result.mode);
}

static void test_crc32_is_crc32_mpeg2()
{
    TEST_ASSERT_EQUAL_HEX32(0x0376E6E7, calculateCRC32((const uint8_t *)"123456789", 9));
    TEST_ASSERT_EQUAL_HEX32(0xFFFFFFFF, calculateCRC32((const uint8_t *)"", 0));
}

static void test_first_boot_connects_regularly_publishes_and_saves_the_access_point()
{
    Wake result = wake();

    assertSleptOnePeriodWithoutRadio(result);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.wifiBegins);
    TEST_ASSERT_EQUAL_INT(0, fake::boot.quickBegins);
    TEST_ASSERT_EQUAL_INT(WIFI_STA, fake::boot.wifiMode);
    TEST_ASSERT_FALSE(fake::boot.persistent);
    TEST_ASSERT_EQUAL_HEX32(ip.value, fake::boot.ip);
    TEST_ASSERT_FALSE(fake::boot.ssid.empty());
    TEST_ASSERT_FALSE(fake::boot.hostname.empty());
    TEST_ASSERT_EQUAL_STRING(fake::boot.hostname.c_str(), fake::boot.clientId.c_str());
    TEST_ASSERT_FALSE(fake::boot.mqttServer.empty());
    TEST_ASSERT_NOT_EQUAL(0, fake::boot.mqttPort);
    TEST_ASSERT_FALSE(fake::boot.otaBegun);
    TEST_ASSERT_GREATER_OR_EQUAL(1, fake::boot.disconnects);

    // The sensor gets power only while it is read
    TEST_ASSERT_EQUAL_INT(1, fake::boot.htuBegins);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.pinHighWrites[SENSOR_VCC]);
    TEST_ASSERT_EQUAL_INT(LOW, fake::boot.pinLevels[SENSOR_VCC]);

    TEST_ASSERT_EQUAL_INT(1, fake::boot.publishes);
    TEST_ASSERT_FALSE(fake::boot.topic.empty());
    StaticJsonDocument<512> doc = published();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 21.5, doc["temperature"].as<float>());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 45.3, doc["humidity"].as<float>());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 4.0, doc["batteryVoltage"].as<float>());
    TEST_ASSERT_EQUAL_INT(85, doc["batteryPercentage"].as<int>());
    TEST_ASSERT_EQUAL_STRING("v1.2", doc["firmwareVersion"].as<const char *>());
    TEST_ASSERT_EQUAL_INT(0, doc["connectFailures"].as<int>());

    RtcData rtc = savedRtc();
    TEST_ASSERT_EQUAL_UINT8(1, rtc.apValid);
    TEST_ASSERT_EQUAL_UINT8(fake::world.channel, rtc.channel);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(fake::world.bssid, rtc.ap_mac, 6);
    TEST_ASSERT_EQUAL_UINT8(0, rtc.failures);
    TEST_ASSERT_EQUAL_UINT16(0, rtc.sleepsLeft);
}

static void test_known_access_point_connects_quickly()
{
    saveRtc(knownAccessPoint());

    Wake result = wake();

    assertSleptOnePeriodWithoutRadio(result);
    TEST_ASSERT_LESS_THAN(1000, result.at);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.wifiBegins);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.quickBegins);
    TEST_ASSERT_EQUAL_INT(fake::world.channel, fake::boot.quickChannel);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(fake::world.bssid, fake::boot.quickBssid, 6);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.publishes);
}

static void test_quick_connect_falls_back_to_regular_after_5s_and_saves_the_new_access_point()
{
    RtcData old = knownAccessPoint();
    old.channel = 11;
    old.ap_mac[5] = 0x99;
    saveRtc(old);
    fake::world.quickConnectWorks = false;

    Wake result = wake();

    assertSleptOnePeriodWithoutRadio(result);
    TEST_ASSERT_GREATER_THAN(5000, result.at);
    TEST_ASSERT_EQUAL_INT(2, fake::boot.wifiBegins);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.quickBegins);
    TEST_ASSERT_EQUAL_INT(11, fake::boot.quickChannel);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.publishes);
    RtcData rtc = savedRtc();
    TEST_ASSERT_EQUAL_UINT8(fake::world.channel, rtc.channel);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(fake::world.bssid, rtc.ap_mac, 6);
}

static void test_corrupt_rtc_counts_as_a_first_boot()
{
    saveRtc(knownAccessPoint(3, 5));
    fake::rtc[5] ^= 0xFF;

    Wake result = wake();

    assertSleptOnePeriodWithoutRadio(result);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.wifiBegins);
    TEST_ASSERT_EQUAL_INT(0, fake::boot.quickBegins);
    TEST_ASSERT_EQUAL_INT(0, published()["connectFailures"].as<int>());
}

static void test_unreadable_rtc_counts_as_a_first_boot()
{
    saveRtc(knownAccessPoint(0, 3));
    fake::world.rtcReadFails = true;

    Wake result = wake();

    assertSleptOnePeriodWithoutRadio(result);
    TEST_ASSERT_EQUAL_INT(0, fake::boot.quickBegins);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.publishes);
}

static void test_wifi_timeout_counts_a_failure_and_waits_two_periods()
{
    fake::world.wifiConnectAfterMs = -1;

    Wake result = wake();

    assertSleptOnePeriodWithoutRadio(result);
    TEST_ASSERT_GREATER_THAN(20000, result.at);
    TEST_ASSERT_LESS_THAN(20100, result.at);
    TEST_ASSERT_EQUAL_INT(0, fake::boot.publishes);
    TEST_ASSERT_GREATER_OR_EQUAL(1, fake::boot.disconnects);
    RtcData rtc = savedRtc();
    TEST_ASSERT_EQUAL_UINT8(1, rtc.failures);
    TEST_ASSERT_EQUAL_UINT16(1, rtc.sleepsLeft);
    TEST_ASSERT_EQUAL_UINT8(0, rtc.apValid);
}

static void test_failures_back_off_10_20_40_80_120_120_minutes()
{
    fake::world.wifiConnectAfterMs = -1;
    std::vector<int> attempts;

    for (int sleeps = 0; sleeps < 200 && attempts.size() < 7; sleeps++)
    {
        Wake result = wake();
        assertSleptOnePeriodWithoutRadio(result); // no single sleep is ever longer
        if (fake::boot.wifiBegins > 0)
        {
            attempts.push_back(sleeps);
        }
        else
        {
            // A wake that only sits out the wait powers neither the sensor nor WiFi
            TEST_ASSERT_EQUAL_UINT32(0, result.at);
            TEST_ASSERT_EQUAL_INT(0, fake::boot.htuBegins);
            TEST_ASSERT_EQUAL_INT(0, fake::boot.pinHighWrites[SENSOR_VCC]);
            TEST_ASSERT_EQUAL_INT(WIFI_OFF, fake::boot.wifiMode);
        }
    }

    // 5-minute periods between two attempts
    const int expected[] = {2, 4, 8, 16, 24, 24};
    TEST_ASSERT_EQUAL_size_t(7, attempts.size());
    for (size_t n = 0; n < 6; n++)
    {
        TEST_ASSERT_EQUAL_INT(expected[n], attempts[n + 1] - attempts[n]);
    }
}

static void test_skip_wake_counts_down_then_tries_again()
{
    saveRtc(knownAccessPoint(1, 1));

    Wake skipped = wake();
    assertSleptOnePeriodWithoutRadio(skipped);
    TEST_ASSERT_EQUAL_INT(0, fake::boot.wifiBegins);
    TEST_ASSERT_EQUAL_UINT16(0, savedRtc().sleepsLeft);
    TEST_ASSERT_EQUAL_UINT8(1, savedRtc().failures);

    Wake tried = wake();
    assertSleptOnePeriodWithoutRadio(tried);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.publishes);
}

static void test_recovery_reports_the_failures_once_and_resets_them()
{
    saveRtc(knownAccessPoint(3, 0));

    wake();
    TEST_ASSERT_EQUAL_INT(3, published()["connectFailures"].as<int>());
    TEST_ASSERT_EQUAL_UINT8(0, savedRtc().failures);
    TEST_ASSERT_EQUAL_UINT16(0, savedRtc().sleepsLeft);

    Wake next = wake();
    assertSleptOnePeriodWithoutRadio(next);
    TEST_ASSERT_EQUAL_INT(0, published()["connectFailures"].as<int>());
}

static void test_failure_counter_saturates_at_255()
{
    saveRtc(knownAccessPoint(255, 0));
    fake::world.wifiConnectAfterMs = -1;

    wake();

    RtcData rtc = savedRtc();
    TEST_ASSERT_EQUAL_UINT8(255, rtc.failures);
    TEST_ASSERT_EQUAL_UINT16(23, rtc.sleepsLeft);
    TEST_ASSERT_EQUAL_UINT8(1, rtc.apValid); // the access point is kept
}

static void test_refused_regular_connection_gives_up_at_once()
{
    const int refusals[] = {WL_NO_SSID_AVAIL, WL_CONNECT_FAILED, WL_WRONG_PASSWORD};
    for (int status : refusals)
    {
        fake::reset();
        fake::world.wifiConnectAfterMs = -1;
        fake::world.wifiFailStatus = status;

        Wake result = wake();

        assertSleptOnePeriodWithoutRadio(result);
        TEST_ASSERT_LESS_THAN(10, result.at);
        TEST_ASSERT_EQUAL_UINT8(1, savedRtc().failures);
    }
}

static void test_refused_quick_connection_still_falls_back_to_regular()
{
    saveRtc(knownAccessPoint());
    fake::world.quickConnectWorks = false;
    fake::world.wifiConnectAfterMs = -1;
    fake::world.wifiFailStatus = WL_NO_SSID_AVAIL;

    Wake result = wake();

    assertSleptOnePeriodWithoutRadio(result);
    TEST_ASSERT_GREATER_THAN(5000, result.at);
    TEST_ASSERT_LESS_THAN(5100, result.at);
    TEST_ASSERT_EQUAL_INT(2, fake::boot.wifiBegins);
    TEST_ASSERT_EQUAL_UINT8(1, savedRtc().failures);
}

static void test_unreachable_broker_counts_a_failure()
{
    fake::world.mqttReachable = false;

    Wake result = wake();

    assertSleptOnePeriodWithoutRadio(result);
    TEST_ASSERT_GREATER_THAN(25000, result.at);
    TEST_ASSERT_LESS_THAN(25100, result.at);
    TEST_ASSERT_GREATER_THAN(1, fake::boot.mqttConnects);
    TEST_ASSERT_EQUAL_INT(0, fake::boot.publishes);
    RtcData rtc = savedRtc();
    TEST_ASSERT_EQUAL_UINT8(1, rtc.failures);
    TEST_ASSERT_EQUAL_UINT16(1, rtc.sleepsLeft);
}

static void test_low_battery_sleeps_for_good()
{
    fake::world.adc = 825; // 3.3 V

    Wake result = wake();

    TEST_ASSERT_TRUE(result.slept);
    TEST_ASSERT_EQUAL_UINT64(0, result.us);
    TEST_ASSERT_EQUAL_INT(WAKE_RF_DEFAULT, result.mode);
    TEST_ASSERT_EQUAL_INT(0, fake::boot.publishes);
}

static void test_battery_just_above_the_cutoff_keeps_going()
{
    fake::world.adc = 826;

    Wake result = wake();

    assertSleptOnePeriodWithoutRadio(result);
    StaticJsonDocument<512> doc = published();
    TEST_ASSERT_FLOAT_WITHIN(0.001, 3.3, doc["batteryVoltage"].as<float>());
    TEST_ASSERT_EQUAL_INT(0, doc["batteryPercentage"].as<int>());
}

static void test_battery_percentage_is_clamped_to_0_and_100()
{
    const struct
    {
        uint16_t adc;
        int percentage;
    } cases[] = {{850, 0}, {862, 0}, {863, 0}, {879, 10}, {1022, 99}, {1023, 100}, {1024, 100}};
    for (const auto &c : cases)
    {
        fake::reset();
        fake::world.adc = c.adc;

        wake();

        TEST_ASSERT_EQUAL_INT_MESSAGE(c.percentage, published()["batteryPercentage"].as<int>(), fake::boot.payload.c_str());
    }
}

static void test_payload_rounds_and_fits_the_longest_values()
{
    saveRtc(knownAccessPoint(255, 0));
    fake::world.temperature = -12.34f;
    fake::world.humidity = 99.96f;
    fake::world.adc = 1024;

    wake();

    TEST_ASSERT_LESS_THAN(191, fake::boot.payload.size());
    StaticJsonDocument<512> doc = published();
    TEST_ASSERT_FLOAT_WITHIN(0.001, -12.3, doc["temperature"].as<float>());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 100.0, doc["humidity"].as<float>());
    TEST_ASSERT_FLOAT_WITHIN(0.001, 4.1, doc["batteryVoltage"].as<float>());
    TEST_ASSERT_EQUAL_INT(100, doc["batteryPercentage"].as<int>());
    TEST_ASSERT_EQUAL_INT(255, doc["connectFailures"].as<int>());
    TEST_ASSERT_EQUAL_STRING("v1.2", doc["firmwareVersion"].as<const char *>());
}

static void test_ota_mode_ignores_the_wait_and_stays_awake()
{
    saveRtc(knownAccessPoint(2, 3));
    fake::world.highInputs = 1u << OTA_PIN;

    Wake result = wake(3000);

    TEST_ASSERT_FALSE(result.slept);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.wifiBegins);
    TEST_ASSERT_TRUE(fake::boot.otaBegun);
    TEST_ASSERT_EQUAL_STRING(fake::boot.hostname.c_str(), fake::boot.otaHostname.c_str());
    TEST_ASSERT_GREATER_THAN(2000, fake::boot.otaHandles);
    TEST_ASSERT_EQUAL_INT(1, fake::boot.publishes);
    // The LED blinks
    TEST_ASSERT_GREATER_THAN(0, fake::boot.pinHighWrites[LED_BUILTIN]);
    TEST_ASSERT_GREATER_THAN(0, fake::boot.pinLowWrites[LED_BUILTIN]);
    // A working connection in OTA mode ends the wait as well
    RtcData rtc = savedRtc();
    TEST_ASSERT_EQUAL_UINT8(0, rtc.failures);
    TEST_ASSERT_EQUAL_UINT16(0, rtc.sleepsLeft);
}

static void test_ota_mode_checks_the_battery_every_1024_loops()
{
    fake::world.highInputs = 1u << OTA_PIN;
    TEST_ASSERT_FALSE(wake(3000).slept);

    fake::world.adc = 800;
    Wake result = keepRunning(2000);

    TEST_ASSERT_TRUE(result.slept);
    TEST_ASSERT_EQUAL_UINT64(0, result.us);
    TEST_ASSERT_LESS_OR_EQUAL(1024, result.loops);
}

void runFirmwareTests()
{
    RUN_TEST(test_crc32_is_crc32_mpeg2);
    RUN_TEST(test_first_boot_connects_regularly_publishes_and_saves_the_access_point);
    RUN_TEST(test_known_access_point_connects_quickly);
    RUN_TEST(test_quick_connect_falls_back_to_regular_after_5s_and_saves_the_new_access_point);
    RUN_TEST(test_corrupt_rtc_counts_as_a_first_boot);
    RUN_TEST(test_unreadable_rtc_counts_as_a_first_boot);
    RUN_TEST(test_wifi_timeout_counts_a_failure_and_waits_two_periods);
    RUN_TEST(test_failures_back_off_10_20_40_80_120_120_minutes);
    RUN_TEST(test_skip_wake_counts_down_then_tries_again);
    RUN_TEST(test_recovery_reports_the_failures_once_and_resets_them);
    RUN_TEST(test_failure_counter_saturates_at_255);
    RUN_TEST(test_refused_regular_connection_gives_up_at_once);
    RUN_TEST(test_refused_quick_connection_still_falls_back_to_regular);
    RUN_TEST(test_unreachable_broker_counts_a_failure);
    RUN_TEST(test_low_battery_sleeps_for_good);
    RUN_TEST(test_battery_just_above_the_cutoff_keeps_going);
    RUN_TEST(test_battery_percentage_is_clamped_to_0_and_100);
    RUN_TEST(test_payload_rounds_and_fits_the_longest_values);
    RUN_TEST(test_ota_mode_ignores_the_wait_and_stays_awake);
    RUN_TEST(test_ota_mode_checks_the_battery_every_1024_loops);
}
