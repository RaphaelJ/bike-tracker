#include <cmath>
#include <climits>

#include <Arduino.h>

#include "gps.hpp"
#include "leds.hpp"
#include "logger.hpp"
#include "movement.hpp"
#include "radio.hpp"

namespace bike_tracker {

class bike_tracker_t {
public:
    enum class state_t { TRACKING, POWER_SAVE };

    static constexpr bool DEBUG = false;

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

    // When in POWER_SAVE mode, probes the location every 15 minutes, movements every 30 seconds (
    // for 1 second), and send the location every 30 minutes. 
    static constexpr uint32_t POWER_SAVE_GPS_PROBE_DELAY = 60 * 60 * 1000;  // millisec
    static constexpr uint32_t POWER_SAVE_RADIO_DELAY     = 60 * 60 * 1000;  // millisec

    void setup()
    {
        led_t::setup_all();

        for (led_t *led : led_t::all) { 
            led->on();
        }

        gps_.instance.setup();
        radio_.instance.setup();
        movement_.detector.setup();

        for (led_t *led : led_t::all) { 
            led->off();
        }
    }

    void loop()
    {
        led_t::blue.on(); // Blue LED in on during when the controller is awake.

        unsigned long millis_current = millis();
        if (millis_prev_ > millis_current) {
            logger::warning("millis() overflowed.");
            millis_offset_ += (uint64_t) ULONG_MAX;

        }
        millis_prev_ = millis_current;
        uint64_t now = (uint64_t) millis_current + millis_offset_;

        switch (state_) {
        case state_t::TRACKING:
            loop_tracking(now);
            break;
        case state_t::POWER_SAVE:
            loop_power_save(now);
            break;
        }
    }

private:
    state_t state_{state_t::TRACKING};

    // Value to add to `millis()` to obtain the actual time of the board. This is required because
    // of overflows and deep sleep events.
    uint64_t millis_offset_{0};

    unsigned long millis_prev_{0};

    struct {
        gps_t instance;

        unsigned long last_probe_try_time{0};

        bool has_position{false}; // false until we get at least on successful GPS position.
        uint64_t last_position_time{0};
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

        uint64_t last_msg_time{0};
    } radio_;

    struct {
        movement_detector_t detector{A1};

        uint64_t last_detection_time{0};
    } movement_;

    void loop_tracking(uint64_t now)
    {
        if (
            now - gps_.last_probe_try_time >= GPS_RETRY_DELAY && (
                !gps_.has_position ||
                now - gps_.last_position_time >= TRACKING_GPS_PROBE_DELAY
            )) {
            probe_gps(now);
        }

        if (now - radio_.last_msg_time >= TRACKING_RADIO_DELAY) {
            send_location_msg(now);
        }

        if (gps_.n_idle >= TRACKING_IDLE_PROBES) {
            // Idle for to much time, go to power save.
            to_power_save();
        } else {
            // Sleep until the next event.
            unsigned long sleep_duration_to_next_gps_probe =
                GPS_RETRY_DELAY - (now - gps_.last_probe_try_time);

            if (gps_.has_position) {
                sleep_duration_to_next_gps_probe = max(
                    sleep_duration_to_next_gps_probe,
                    TRACKING_GPS_PROBE_DELAY - (now - gps_.last_position_time)
                );
            }

            unsigned long sleep_duration = min(
                sleep_duration_to_next_gps_probe,
                TRACKING_RADIO_DELAY - (now - radio_.last_msg_time)
            );

            sleep(sleep_duration);
        }
    }

    void loop_power_save(uint64_t now)
    {
        if (movement_.detector.detected()) {
            // Movement detected, go to live tracking.
            logger::info("Movement detected.");
            to_tracking();
            return;
        }

        if (
            now - gps_.last_probe_try_time >= GPS_RETRY_DELAY && (
                !gps_.has_position ||
                now - gps_.last_position_time >= POWER_SAVE_GPS_PROBE_DELAY)) {
            bool success = probe_gps(now);
            if (success) {
                gps_.instance.sleep();
            }
        }

        if (now - radio_.last_msg_time >= POWER_SAVE_RADIO_DELAY) {
            send_location_msg(now);
        }

        if (gps_.n_idle == 0) {
            logger::info("GPS movement detected.");
            to_tracking();
        } else {
            // Sleep until the next event.
            unsigned long sleep_duration_to_next_gps_probe =
                GPS_RETRY_DELAY - (now - gps_.last_probe_try_time);

            if (gps_.has_position) {
                sleep_duration_to_next_gps_probe = max(
                    sleep_duration_to_next_gps_probe,
                    POWER_SAVE_GPS_PROBE_DELAY - (now - gps_.last_position_time)
                );
            }

            unsigned long sleep_duration = min(
                sleep_duration_to_next_gps_probe,
                POWER_SAVE_RADIO_DELAY - (now - radio_.last_msg_time)
            );

            sleep(sleep_duration);
        }
    }

    void to_tracking()
    {
        logger::info("Entering live tracking state");

        state_ = state_t::TRACKING;
        gps_.n_idle = 0;
        gps_.instance.wake_up();

        movement_.detector.disable();

        led_t::builtin.on();
    }

    void to_power_save()
    {
        logger::info("Entering power save state");

        state_ = state_t::POWER_SAVE;
        gps_.instance.sleep();

        movement_.detector.reset();
        movement_.detector.enable();
        led_t::builtin.off();
    }

    // Sleep into a low power sleep mode for the given number of milliseconds.
    void sleep(unsigned long duration)
    {
        logger::info("Sleep for " + String(duration) + " ms");
        led_t::blue.off();

        LowPower.sleep(duration);
        millis_offset_ += duration; // TODO: checks if the board wake up because of the RTC.

        led_t::blue.off();
        logger::info("Sleep ended");
    }

    // Tries to get the current position.
    //
    // Returns `true` on success.
    bool probe_gps(unsigned long time)
    {
        gps_t::position_t position = gps_.instance.get_position();

        bool success = position.has_gnss_fix && position.n_satellites > 0;

        if (success) {
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

        return success;
    }

    void send_location_msg(uint64_t now)
    {
        logger::info("Send location message");

        float lat, lng;

        if (gps_.has_position && gps_.last_position_time > radio_.last_msg_time) {
            lat = gps_.last_position.coordinates.lat;
            lng = gps_.last_position.coordinates.lng;
        } else {
            logger::warning("\tNo new location update.");
            lat = 0.0f;
            lng = 0.0f;
        }

        uint32_t last_msg = now - radio_.last_msg_time;

        radio_t::location_msg_t msg{
            lat, lng, gps_.distance, gps_.alt_gain, gps_.max_speed, last_msg};

        radio_.instance.send(msg);

        gps_.distance = 0.0f;
        gps_.alt_gain = 0.0f;
        gps_.max_speed = 0.0f;

        radio_.last_msg_time = now;
    }
};

} // namespace bike_tracker

