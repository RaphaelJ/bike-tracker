#pragma once

#include <Arduino.h>

#include "leds.hpp"

namespace bike_tracker::logger {

void error(const String &msg)
{
    Serial.print("[ERROR]   ");
    Serial.println(msg);

    // Blinks all the leds and never return.
    bool leds_on = true;
    for (;;) {
        for (led_t *led : led_t::all) {
            led->set(leds_on);
        }

        delay(500);
        leds_on = !leds_on;
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