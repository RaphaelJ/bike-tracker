#pragma once

#include <climits>

#include <Arduino.h>
#include <ArduinoLowPower.h>

#include "logger.hpp"

namespace bike_tracker {

class movement_detector_t {

public:
    movement_detector_t(pin_size_t pin_num) : pin_num_(pin_num)
    { }

    void setup()
    {
        pinMode(pin_num_, INPUT);
    }

    /** Enables a asynchronous movement detection. */
    void enable()
    {
        logger::info("Enable asynchronous movement detection");
        attachInterrupt(digitalPinToInterrupt(pin_num_), on_interrupt, LOW);
    }

    /** Disables asynchronous movement detector connected on pin `pin_num`. */
    void disable()
    {
        logger::info("Disable asynchronous movement detection");
        detachInterrupt(digitalPinToInterrupt(pin_num_));
    }

    /** Returns true if some movement has been detected since the last call to `reset()`. */
    static bool detected()
    {
        return detected_;
    }

    static void reset()
    {
        logger::info("Reset asynchronous movement detection (was: " + String(detected()) + ")");
        detected_ = false;
    }

private:
    static volatile bool detected_;

    pin_size_t pin_num_;

    static void on_interrupt()
    {
        detected_ = true;
    }
};

volatile bool movement_detector_t::detected_{false};

}