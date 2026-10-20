#pragma once

#include <Arduino.h>

class Adafruit_HTU21DF
{
public:
    bool begin();
    float readTemperature();
    float readHumidity();
};
