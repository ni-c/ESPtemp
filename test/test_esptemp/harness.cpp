#include "harness.h"

#include <unity.h>

void setUp() { fake::reset(); }

void tearDown() {}

Wake keepRunning(long maxLoops)
{
    for (long loops = 1; loops <= maxLoops; loops++)
    {
        try
        {
            loop();
        }
        catch (const fake::DeepSleep &sleep)
        {
            return {true, sleep.us, sleep.mode, fake::boot.now, loops};
        }
        fake::boot.now += 1;
    }
    return {false, 0, 0, fake::boot.now, maxLoops};
}

Wake wake(long maxLoops)
{
    fake::newBoot();
    try
    {
        setup();
    }
    catch (const fake::DeepSleep &sleep)
    {
        return {true, sleep.us, sleep.mode, fake::boot.now, 0};
    }
    return keepRunning(maxLoops);
}

RtcData savedRtc()
{
    RtcData data;
    memcpy(&data, fake::rtc, sizeof(data));
    return data;
}

void saveRtc(RtcData data)
{
    data.crc32 = calculateCRC32(((uint8_t *)&data) + 4, sizeof(data) - 4);
    memcpy(fake::rtc, &data, sizeof(data));
}

RtcData knownAccessPoint(uint8_t failures, uint16_t sleepsLeft)
{
    RtcData data = {};
    data.channel = fake::world.channel;
    memcpy(data.ap_mac, fake::world.bssid, 6);
    data.apValid = 1;
    data.failures = failures;
    data.sleepsLeft = sleepsLeft;
    return data;
}
