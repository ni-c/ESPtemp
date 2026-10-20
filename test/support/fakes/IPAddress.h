#pragma once

#include <cstdint>

class IPAddress
{
public:
    IPAddress() : value(0) {}
    IPAddress(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
        : value(((uint32_t)a << 24) | ((uint32_t)b << 16) | ((uint32_t)c << 8) | d) {}

    uint32_t value;
};
