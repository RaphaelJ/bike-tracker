#pragma once

#include <Arduino.h>

#include "logger.hpp"

namespace bike_tracker {

class button_t {

    button_t(pin_size_t pin_num) : pin_num_(pin_num) { }

public:
    static button_t a;

    static void setup_all()
    {   
        a.setup();
    }

    bool pressed()
    {
        return digitalRead(pin_num_) == LOW;
    }

    void setup()
    {
        pinMode(pin_num_, INPUT_PULLUP);
    }

private:
    pin_size_t pin_num_;
};

button_t button_t::a{2};

}