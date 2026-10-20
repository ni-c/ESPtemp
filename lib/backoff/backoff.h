#pragma once

#include <stdint.h>

/**
 * Number of whole sleep periods that fit into the longest wait, at least one.
 */
inline uint16_t backoffCap(double maxSleepUs, double sleepUs)
{
    if (sleepUs <= 0 || maxSleepUs < sleepUs)
    {
        return 1;
    }
    double periods = maxSleepUs / sleepUs;
    return periods > 65535 ? 65535 : (uint16_t)periods;
}

/**
 * Sleep periods to wait before the next connection attempt after `failures`
 * failed attempts in a row: 1, 2, 4, 8, ... up to `cap`.
 */
inline uint16_t backoffSleeps(uint8_t failures, uint16_t cap)
{
    if (cap < 1)
    {
        cap = 1;
    }
    if (failures >= 16)
    {
        return cap;
    }
    uint32_t periods = 1UL << failures;
    return periods < cap ? (uint16_t)periods : cap;
}
