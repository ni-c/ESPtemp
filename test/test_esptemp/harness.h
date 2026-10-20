#pragma once

#include <fake.h>

#include "rtc_data.h"

// From src/main.cpp
void setup();
void loop();
uint32_t calculateCRC32(const uint8_t *data, size_t length);

struct Wake
{
    bool slept;       // false: still awake when the loop limit was reached (OTA wait loop)
    uint64_t us;      // deep sleep time, 0 is forever
    int mode;         // RFMode of the deep sleep
    unsigned long at; // millis() when it went to sleep
    long loops;       // loop() calls in this run
};

// A wake from deep sleep: setup(), then loop() until the next deep sleep.
Wake wake(long maxLoops = 100000);
// More loop() calls in the same boot.
Wake keepRunning(long maxLoops);

RtcData savedRtc();
// Stores `data` in RTC memory with a valid CRC.
void saveRtc(RtcData data);
// Data for an access point the device has connected to before.
RtcData knownAccessPoint(uint8_t failures = 0, uint16_t sleepsLeft = 0);
