#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <string>

#include "fake.h"

#define HIGH 0x1
#define LOW 0x0
#define INPUT 0x00
#define OUTPUT 0x01
#define LED_BUILTIN 2
#define A0 17

unsigned long millis();
void delay(unsigned long ms);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int digitalRead(uint8_t pin);
int analogRead(uint8_t pin);

class String : public std::string
{
public:
    using std::string::string;
    String() = default;
    String(const std::string &value) : std::string(value) {}
};

enum RFMode
{
    RF_DEFAULT = 0,
    RF_CAL = 1,
    RF_NO_CAL = 2,
    RF_DISABLED = 4
};
#define WAKE_RF_DEFAULT RF_DEFAULT
#define WAKE_RF_DISABLED RF_DISABLED

class EspClass
{
public:
    void deepSleep(uint64_t timeUs, RFMode mode = RF_DEFAULT);
    bool rtcUserMemoryRead(uint32_t offset, uint32_t *data, size_t size);
    bool rtcUserMemoryWrite(uint32_t offset, uint32_t *data, size_t size);
    uint8_t getCpuFreqMHz();
};

extern EspClass ESP;
