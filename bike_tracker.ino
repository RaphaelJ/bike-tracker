#include <Arduino.h>
#include <ArduinoLowPower.h>

#include "bike_tracker.hpp"

bike_tracker::bike_tracker_t tracker;

void setup()
{
    tracker.setup();
}

void loop()
{
    tracker.loop();
}