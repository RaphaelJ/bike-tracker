#pragma once

#include <Arduino.h>

#include "leds.hpp"

namespace bike_tracker::logger {

void error(const String &msg)
{
    Serial.print("[ERROR]   ");
    Serial.println(msg);

    for (;;) {
        led_t::red.on();
        delay(500);
        led_t::red.off();
        delay(500);
    }
}

void warning(const String &msg)
{
    Serial.print("[WARNING] ");
    Serial.println(msg);
}

void info(const String &msg)
{
    Serial.print("[INFO]    ");
    Serial.println(msg);
}

}