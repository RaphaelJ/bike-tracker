#include <cmath>

#include <Arduino.h>

#include "buttons.hpp"
#include "gps.hpp"
#include "leds.hpp"
#include "logger.hpp"
#include "radio.hpp"

namespace bike_tracker {

class bike_tracker_t {
public:
    enum class state_t { TRACKING, POWER_SAVE };

    // Will consider the bike idle if it moves slower than 3kph between two GPS probes.
    static constexpr float IDLE_THRESHOLD = 5.0f * 1000.0f / 3600.0f; // meters per sec

    // Will wait 1 second after an unsuccessful GPS probe before trying again.
    static constexpr uint32_t GPS_RETRY_DELAY = 5 * 1000; // millisec

    // Smooth the GPS altitude probes by using a recursive smoother.
    static constexpr float GPS_ALT_SMOOTHER_FACTOR = 0.2f; // millisec

    // When in the TRACKING state, probes the location and speed every 30 seconds, and sends the
    // location every 5 minutes. The tracker will move into the POWER_SAVE state if there the sensor
    // stayed idle for 10 location probes.
    static constexpr uint32_t TRACKING_GPS_PROBE_DELAY  = 30 * 1000;        // millisec
    static constexpr uint32_t TRACKING_RADIO_DELAY      = 5 * 60 * 1000;    // millisec
    static constexpr uint32_t TRACKING_IDLE_PROBES      = 5;

    // When in POWER_SAVE mode, probes the location every 15 minutes, movements every 2 minutes (for
    // 3 seconds), and send the location every 30 minutes. 
    // static constexpr uint32_t POWER_SAVE_GPS_PROBE_DELAY = 15 * 60 * 1000;  // millisec
    static constexpr uint32_t POWER_SAVE_GPS_PROBE_DELAY = 60 * 1000;  // millisec
    static constexpr uint32_t POWER_SAVE_MOVEMENT_DELAY  = 2 * 60 * 1000;   // millisec
    static constexpr uint32_t POWER_SAVE_MOVEMENT_WAIT   = 3 * 1000;        // millisec
    static constexpr uint32_t POWER_SAVE_RADIO_DELAY     = 30 * 60 * 1000;  // millisec

    void setup()
    {
        button_t::setup_all();
        led_t::setup_all();

        for (led_t *led : led_t::all) { 
            led->on();
        }

        gps_.instance.setup();
        radio_.instance.setup();

        reset(millis());

        for (led_t *led : led_t::all) { 
            led->off();
        }
    }

    void loop()
    {
        led_t::blue.on(); // Blue LED in on during when the controller is awake.

        unsigned long time = millis();
        if (prev_time > time) {
            // millis() overflowed.
            // We can not trust the internal time anymore, we reset the tracker internal states.
            logger::warning("millis() overflowed. Reset tracker.");
            reset(time);
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

    struct {
        gps_t instance;

        unsigned long last_probe_try_time{0};

        bool has_position{false}; // false until we get at least on successful GPS position.
        unsigned long last_position_time{0};
        gps_t::position_t last_position;
        float smoothed_alt;

        // Accumulated since the last location message:
        float distance{0};  // meters
        float alt_gain{0};  // meters
        float max_speed{0}; // m/s 

        // The number of previous GPS probes that did not exceed IDLE_THRESHOLD.
        uint32_t n_idle{0};
    } gps_;

    struct {
        radio_t instance;

        unsigned long last_msg_time{0};
    } radio_;

    void reset(unsigned long time)
    {
        state_ = state_t::TRACKING;
        prev_time = 0;

        gps_.has_position = false;
        gps_.last_position_time = 0;
        gps_.smoothed_alt = 0;

        gps_.distance = 0;
        gps_.alt_gain = 0;
        gps_.max_speed = 0;

        gps_.n_idle = 0;

        radio_.last_msg_time = time;

        to_tracking();
    }

    void loop_tracking(unsigned long time)
    {
        if (
            time - gps_.last_probe_try_time >= GPS_RETRY_DELAY && (
                !gps_.has_position ||
                time - gps_.last_position_time >= TRACKING_GPS_PROBE_DELAY
            )) {
            probe_gps(time);
        }

        if (time - radio_.last_msg_time >= TRACKING_RADIO_DELAY) {
            send_location_msg(time);
        }

        if (gps_.n_idle >= TRACKING_IDLE_PROBES) {
            // Idle for to much time, go to power save.
            to_power_save();
        }
    }

    void loop_power_save(unsigned long time)
    {
        if (
            time - gps_.last_probe_try_time >= GPS_RETRY_DELAY && (
                !gps_.has_position ||
                time - gps_.last_position_time >= POWER_SAVE_GPS_PROBE_DELAY)) {
            probe_gps(time);
        }

        if (time - radio_.last_msg_time >= POWER_SAVE_RADIO_DELAY) {
            send_location_msg(time);
        }

        if (gps_.n_idle == 0) {
            // Movement detected, go to live tracking.
            to_tracking();
        }
    }

    void to_tracking()
    {
        logger::info("Entering live tracking state");

        state_ = state_t::TRACKING;
        gps_.instance.powerSave(false);
    }

    void to_power_save()
    {
        logger::info("Entering power save state");

        state_ = state_t::POWER_SAVE;
        gps_.instance.powerSave(true);
    }

    // Tries to get the current position.
    void probe_gps(unsigned long time)
    {
        gps_t::position_t position = gps_.instance.get_position();

        if (position.has_gnss_fix && position.n_satellites > 0) {
            logger::info("New GPS probe");
            logger::info(
                "\tLat.: " + String(position.coordinates.lat, 6) + " - " +
                "Long.: " + String(position.coordinates.lng, 6) + " - " +
                "Alt.: " + String(position.coordinates.alt, 2) + "m - " +
                "Sats: " + String(position.n_satellites));

            if (gps_.has_position) {
                float delta_secs = 
                    ((float) (time - gps_.last_position_time)) / 1000.0f; // secs
                float dist_meters = gps_t::distance(
                    position.coordinates, gps_.last_position.coordinates);

                float speed_ms = dist_meters / delta_secs;

                float smoothed_alt = 
                    gps_.smoothed_alt * (1.0f - GPS_ALT_SMOOTHER_FACTOR) +
                    position.coordinates.alt * GPS_ALT_SMOOTHER_FACTOR;

                float alt_gain =
                    smoothed_alt > gps_.smoothed_alt ?
                    smoothed_alt - gps_.smoothed_alt :
                    0;

                bool is_idle = speed_ms < IDLE_THRESHOLD;

                logger::info(
                    "\tDistance: " + String(dist_meters, 2) + "m - " +
                    "Speed: " + String(speed_ms, 2) + "m/s  - " +
                    "Alt. gain: " + String(alt_gain, 2) + "m - " +
                    "Idle: " + String(is_idle));

                if (is_idle) {
                    ++gps_.n_idle;
                } else {
                    gps_.distance += dist_meters;
                    gps_.alt_gain += alt_gain;

                    if (speed_ms > gps_.max_speed) {
                        gps_.max_speed = speed_ms;
                    }

                    gps_.n_idle = 0;
                }
            } else {
                gps_.smoothed_alt = position.coordinates.alt;
            }

            gps_.has_position = true;
            gps_.last_position_time = time;
            gps_.last_position = position;
        } else {
            logger::warning("Unsuccessful GPS probe.");
        }

        gps_.last_probe_try_time = time;
    }

    void send_location_msg(unsigned long time)
    {
        logger::info("Send location message");

        float lat, lng;

        if (gps_.has_position && gps_.last_position_time > radio_.last_msg_time) {
            lat = gps_.last_position.coordinates.lat;
            lng = gps_.last_position.coordinates.lat;
        } else {
            logger::warning("\tNo new location update.");
            lat = 0.0f;
            lng = 0.0f;
        }

        uint32_t last_msg = time - radio_.last_msg_time;

        radio_t::location_msg_t msg{
            lat, lng, gps_.distance, gps_.alt_gain, gps_.max_speed, last_msg};

        bool success = radio_.instance.send(msg);

        if (success) {
            gps_.distance = 0.0f;
            gps_.alt_gain = 0.0f;
            gps_.max_speed = 0.0f;

            radio_.last_msg_time = time;
        }
    }
};

} // namespace bike_tracker

