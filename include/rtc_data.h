#pragma once

#include <stdint.h>

/**
 * Kept in RTC memory across deep sleep. The CRC covers everything after it.
 */
struct RtcData
{
    uint32_t crc32;      // 4 bytes
    uint8_t channel;     // 1 byte,   5 in total
    uint8_t ap_mac[6];   // 6 bytes, 11 in total
    uint8_t apValid;     // 1 byte,  12 in total: channel and ap_mac are known
    uint8_t failures;    // 1 byte,  13 in total: failed connection attempts in a row
    uint8_t reserved;    // 1 byte,  14 in total
    uint16_t sleepsLeft; // 2 bytes, 16 in total: sleep periods to skip before the next attempt
};

static_assert(sizeof(RtcData) == 16, "RtcData must stay a multiple of 4 bytes");
