#pragma once

#include <Arduino.h>

namespace bike_tracker {

class led_t {

    led_t(pin_size_t pin_num) : pin_num_(pin_num) { }

public:
    static led_t red;
    static led_t green;
    static led_t blue;

    static void setup_all()
    {   
        led_t::red.setup();
        led_t::green.setup();
        led_t::blue.setup();
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

led_t led_t::red{3};
led_t led_t::green{4};
led_t led_t::blue{5};

}