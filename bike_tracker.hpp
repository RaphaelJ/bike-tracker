#include <Arduino.h>

#include "buttons.hpp"
#include "gps.hpp"
#include "leds.hpp"
#include "logger.hpp"

namespace bike_tracker {

class bike_tracker_t {
public:
    enum class state_t { TRACKING, POWER_SAVE };

    // Will consider the bike idle if it moves slower than 3kph between two GPS probes.
    static constexpr float IDLE_THRESHOLD = 3.0f * 1000.0f / 3600.0f; // meters per sec

    // Will wait 1 second after an unsuccessful GPS probe before trying again.
    static constexpr uint32_t GPS_TRY_DELAY = 1 * 1000; // millisec

    // When in the TRACKING state, probes the location and speed every 30 seconds, and sends the
    // location every 5 minutes. The tracker will move into the POWER_SAVE state if there the sensor
    // stayed idle for 10 location probes.
    static constexpr uint32_t TRACKING_GPS_PROBE_DELAY  = 30 * 1000;        // millisec
    static constexpr uint32_t TRACKING_RADIO_MESSAGE    = 5 * 60 * 1000;    // millisec
    static constexpr uint32_t TRACKING_IDLE_PROBES      = 10;

    // When in POWER_SAVE mode, probes the location every 2 minutes, and send the location every 20
    // minutes.
    static constexpr uint32_t POWER_SAVE_GPS_PROBE_DELAY = 2 * 60 * 1000;    // millisec

    void setup()
    {
        button_t::setup_all();
        led_t::setup_all();

        led_t::red.on();
        led_t::green.on();
        led_t::blue.on();

        gps_.setup();

        led_t::red.off();
        led_t::green.off();
        led_t::blue.off();

        reset();
    }

    void loop()
    {
        unsigned long time = millis();
        if (prev_time > time) {
            // millis() overflowed.
            // We can not trust the internal time anymore, we reset the tracker internal states.
            logger::warning("millis() overflowed. Reset tracker.");
            reset();
        }

        switch (state_) {
        case state_t::TRACKING:
            loop_tracking(time);
            break;
        case state_t::POWER_SAVE:
            loop_power_save(time);
            break;
        }

        prev_time = time;
    }

private:
    state_t state_{state_t::TRACKING};

    unsigned long prev_time{0};

    gps_t gps_;

    struct gps_probe_t {
        unsigned long time;
        gps_t::position_t position;
    };
    
    bool has_gps_probe_{false};     // false ultil we get at least on successful GPS probe.
    gps_probe_t last_gps_probe_{};

    // The number of previous GPS probes that did not top the IDLE_THRESHOLD.
    uint32_t n_idle_gps{0};

    void reset()
    {
        state_ = state_t::TRACKING;
        prev_time = 0;
        has_gps_probe_ = 0;
        n_idle_gps = 0;

        to_tracking();
    }

    void loop_tracking(unsigned long time)
    {
        if (!has_gps_probe_ || time - last_gps_probe_.time >= TRACKING_GPS_PROBE_DELAY) {
            probe_gps(time);
        }

        if (n_idle_gps >= TRACKING_IDLE_PROBES) {
            // Idle for to much time, go to power save.
            to_power_save();
        }
    }

    void loop_power_save(unsigned long time)
    {
        if (!has_gps_probe_ || time - last_gps_probe_.time >= POWER_SAVE_GPS_PROBE_DELAY) {
            probe_gps(time);
        }

        if (n_idle_gps != 0) {
            // Movement detected, go to live tracking.
            to_tracking();
        }
    }

    void to_tracking()
    {
        logger::info("Entering live tracking state");

        led_t::blue.on();
        led_t::green.off();

        state_ = state_t::TRACKING;
        n_idle_gps = 0;
        gps_.powerSave(false);
    }

    void to_power_save()
    {
        logger::info("Entering power save state");

        led_t::blue.off();
        led_t::green.on();

        state_ = state_t::POWER_SAVE;
        n_idle_gps = 0;
        gps_.powerSave(true);
    }

    // Tries to get the current position.
    //
    // Returns `true` if the GPS chip returned a new position.
    bool probe_gps(unsigned long time)
    {
        gps_t::position_t position = gps_.get_position();

        if (position.has_gnss_fix) {
            gps_probe_t new_probe{time, position};

            logger::info("New GPS probe");
            logger::info(
                "\tLat.: " + String(position.coordinates.lat, 6) + " - " +
                "Long.: " + String(position.coordinates.lng, 6) + " - " +
                "Alt.: " + String(position.coordinates.alt, 2) + "m - " +
                "Sats: " + String(position.n_satellites));

            if (has_gps_probe_) {
                float delta_secs = ((float) (time - last_gps_probe_.time)) / 1000.0f; // secs
                float dist_meters = gps_t::distance(
                    position.coordinates, last_gps_probe_.position.coordinates);

                float speed_ms = dist_meters / delta_secs * 3600.0f;

                bool is_idle = speed_ms < IDLE_THRESHOLD;

                logger::info(
                    "\tDistance: " + String(dist_meters, 2) + "m - " +
                    "Speed: " + String(speed_ms, 2) + "m/s  - " +
                    "Idle: " + String(is_idle));

                if (is_idle) {
                    ++n_idle_gps;
                }
            }

            has_gps_probe_ = true;
            last_gps_probe_ = new_probe;
        } else {
            logger::warning("Unsuccessful GPS probe.");
        }
    }
};

} // namespace bike_tracker

