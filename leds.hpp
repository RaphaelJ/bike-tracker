#pragma once

#include <Arduino.h>

namespace bike_tracker {

class led_t {

    led_t(pin_size_t pin_num) : pin_num_(pin_num) { }

public:
    static led_t builtin;
    static led_t blue;

    // Contains a pointer to all the led instances in the board.
    static led_t *all[2];

    static void setup_all()
    {   
        for (led_t *led : all) {
            led->setup();
        }
    }

    void setup()
    {
        pinMode(pin_num_, OUTPUT);
    }

    void set(bool on)
    {
        digitalWrite(pin_num_, on ? HIGH : LOW);
    }

    void on()
    {
        set(true);
    }

    void off()
    {
        set(false);
    }

private:
    pin_size_t pin_num_;
};

led_t led_t::builtin{LED_BUILTIN};
led_t led_t::blue{5};

led_t *led_t::all[2] {&builtin, &blue};

}