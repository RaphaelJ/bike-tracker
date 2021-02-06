#include <climits>
#include <cmath>
#include <ctime>

#include <Arduino.h>
#include <RTCZero.h>
#include <Embedded_Template_Library.h>
#include <etl/queue.h>

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
    static constexpr uint32_t GPS_RETRY_DELAY = 5; // sec

    // Smooth the GPS altitude probes by using a recursive smoother.
    static constexpr float GPS_ALT_SMOOTHER_FACTOR = 0.2f;  // ratio

    // When in the TRACKING state, probes the location and speed every 20 seconds, and sends the
    // location every 3 minutes.
    static constexpr uint32_t TRACKING_GPS_PROBE_DELAY  = 20;        // sec
    static constexpr uint32_t TRACKING_RADIO_DELAY      = 3 * 60;    // sec

    // The tracker will move into the POWER_SAVE state if there the sensor stayed idle for 9 of the
    // last 12 location probes (4 minutes).
    static constexpr uint32_t TRACKING_IDLE_PROBES      = 9;
    static constexpr uint32_t TRACKING_IDLE_BUFFER_SIZE = 12;

    // When in POWER_SAVE mode, probes and transmit the location every 60 minutes. 
    static constexpr uint32_t POWER_SAVE_GPS_PROBE_DELAY = 60 * 60;  // sec

    void setup()
    {
        led_t::setup_all();

        for (led_t *led : led_t::all) { 
            led->on();
        }

        gps_.instance.setup();
        radio_.instance.setup();
        movement_.detector.setup();

        clock_.begin();
        clock_.setY2kEpoch(0);

        for (led_t *led : led_t::all) { 
            led->off();
        }
    }

    void loop()
    {
        led_t::blue.on(); // Blue LED in on during when the controller is awake.

        uint32_t now = clock_.getY2kEpoch();

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

    RTCZero clock_{};

    enum class probe_result_t { NO_FIX, IDLE, MOVING, UNKNOWN };

    struct {
        gps_t instance;

        uint32_t next_probe_time{0};

        bool has_position{false}; // false until we get at least on successful GPS position.
        gps_t::position_t last_position;
        uint32_t last_position_time;
        float smoothed_alt;

        // Accumulated since the last location message:
        float distance{0};  // meters
        float alt_gain{0};  // meters
        float max_speed{0}; // m/s 

        // The number of previous GPS probes that did not exceed IDLE_THRESHOLD.
        //
        // Keep a buffer of the previous probes 
        etl::queue<bool, TRACKING_IDLE_BUFFER_SIZE> idle_probes{};
        uint32_t n_idle{0};
    } gps_;

    struct {
        radio_t instance;

        uint32_t next_msg_time{TRACKING_RADIO_DELAY};
        uint32_t last_msg_time{0};
    } radio_;

    struct {
        movement_detector_t detector{A1};
    } movement_;


    void loop_tracking(uint32_t now)
    {
        if (now >= gps_.next_probe_time) {
            probe_result_t result = probe_gps(now);

            if (result == probe_result_t::NO_FIX) {
                gps_.next_probe_time = now + GPS_RETRY_DELAY;
            } else {
                gps_.next_probe_time = now + TRACKING_GPS_PROBE_DELAY;

                if (result != probe_result_t::UNKNOWN) {
                    if (gps_.idle_probes.full()) {
                        if (gps_.idle_probes.front()) {
                            --gps_.n_idle;
                        }
                        gps_.idle_probes.pop();
                    }

                    bool is_idle = result == probe_result_t::IDLE;

                    if (is_idle) {
                        ++gps_.n_idle;
                    }

                    gps_.idle_probes.push(is_idle);
                }
            }
        }

        if (now >= radio_.next_msg_time) {
            send_location_msg(now);
            radio_.next_msg_time = now + TRACKING_RADIO_DELAY;
        }

        if (gps_.n_idle >= TRACKING_IDLE_PROBES) {
            // Idle for to much time, go to power save.
            to_power_save(now);
        } else {
            // Sleep until the next event.
            unsigned long duration = min(gps_.next_probe_time, radio_.next_msg_time) - now;
            sleep(max(500, duration * 1000));
        }
    }

    void loop_power_save(uint32_t now)
    {
        if (movement_.detector.detected()) {
            // Movement detected, go to live tracking.
            logger::info("Movement detected.");
            to_tracking(now);
            return;
        }

        if (now >= gps_.next_probe_time) {
            probe_result_t result = probe_gps(now);

            if (result == probe_result_t::NO_FIX) {
                gps_.next_probe_time = now + GPS_RETRY_DELAY;
            } else {
                gps_.instance.sleep();
                gps_.next_probe_time = now + POWER_SAVE_GPS_PROBE_DELAY;

                send_location_msg(now);

                if (result != probe_result_t::IDLE) {
                    logger::info("GPS movement detected.");
                    to_tracking(now);
                    return;
                }
            }
        }
        
        // Sleep until the next GPS event.
        unsigned long duration = gps_.next_probe_time - now;
        sleep(max(500, duration * 1000));
    }

    void to_tracking(uint32_t now)
    {
        logger::info("Entering live tracking state");

        state_ = state_t::TRACKING;
        gps_.instance.wake_up();
        gps_.next_probe_time = now;
        gps_.idle_probes.clear();
        gps_.n_idle = 0;

        radio_.next_msg_time = now + TRACKING_RADIO_DELAY;

        movement_.detector.disable();
    }

    void to_power_save(uint32_t now)
    {
        logger::info("Entering power save state");

        state_ = state_t::POWER_SAVE;
        gps_.instance.sleep();

        if (gps_.has_position) {
            gps_.next_probe_time = gps_.last_position_time + POWER_SAVE_GPS_PROBE_DELAY;
        } else {
            gps_.next_probe_time = now + GPS_RETRY_DELAY;
        }

        movement_.detector.reset();
        movement_.detector.enable();
    }

    // Sleep into a low power sleep mode for the given number of milliseconds.
    void sleep(unsigned long duration)
    {
        logger::info("Sleep for " + String(duration) + " ms");
        led_t::blue.off();

        if (!DEBUG || state_ == state_t::POWER_SAVE) {
            LowPower.sleep(duration);
        } else {
            delay(duration);
        }

        led_t::blue.off();
        logger::info("Sleep ended");
    }

    // Tries to get the current position.
    probe_result_t probe_gps(unsigned long now)
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

            probe_result_t result;

            if (gps_.has_position) {
                float delta_secs = (float) (now - gps_.last_position_time);
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
                    result = probe_result_t::IDLE;
                } else {
                    gps_.distance += dist_meters;
                    gps_.alt_gain += alt_gain;

                    if (speed_ms > gps_.max_speed) {
                        gps_.max_speed = speed_ms;
                    }

                    result = probe_result_t::MOVING;
                }
            } else {
                gps_.smoothed_alt = position.coordinates.alt;
                result = probe_result_t::UNKNOWN;
            }

            gps_.has_position = true;
            gps_.last_position = position;
            gps_.last_position_time = now;

            return result;
        } else {
            logger::warning("Unsuccessful GPS probe.");
            return probe_result_t::NO_FIX;
        }
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
