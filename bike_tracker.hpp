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

    static constexpr bool DEBUG = true;

    // Will consider the bike idle if it moves slower than 4kph between two GPS probes.
    static constexpr float IDLE_THRESHOLD = 4.0f * 1000.0f / 3600.0f; // meters per sec

    // Will wait 1 second after an unsuccessful GPS probe before trying again.
    static constexpr uint32_t GPS_RETRY_DELAY = 5; // sec

    // Will wait 1 minute after an unsuccessful radio message before trying again.
    static constexpr uint32_t RADIO_RETRY_DELAY = 60; // sec

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

        uint32_t last_msg_time{0};

        // Undefined if no planned message.
        etl::optional<uint32_t> next_msg_time{TRACKING_RADIO_DELAY};
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

        if (radio_.next_msg_time.has_value() && now >= radio_.next_msg_time) {
            bool success = send_location_msg(now);

            if (success) {
                radio_.next_msg_time = now + TRACKING_RADIO_DELAY;
            } else {
                radio_.next_msg_time = now + RADIO_RETRY_DELAY;
            }
        }

        if (gps_.n_idle >= TRACKING_IDLE_PROBES) {
            // Idle for to much time, go to power save.
            to_power_save(now);
        } else {
            sleep(now);
        }
    }

    void loop_power_save(uint32_t now)
    {
        bool movement{false};

        if (movement_.detector.detected()) {
            // Movement detected, go to live tracking.
            logger::info("Movement detected using movement detector.");
            movement = true;
        }

        if (now >= gps_.next_probe_time) {
            probe_result_t result = probe_gps(now);

            if (result == probe_result_t::NO_FIX) {
                gps_.next_probe_time = now + GPS_RETRY_DELAY;
            } else {
                gps_.instance.sleep();
                gps_.next_probe_time = now + POWER_SAVE_GPS_PROBE_DELAY;

                // Sends the coordinates ASAP.
                radio_.next_msg_time = now;

                if (result != probe_result_t::IDLE) {
                    logger::info("Movement detected using GPS.");
                    movement = true;
                }
            }
        }

        if (radio_.next_msg_time.has_value() && now >= *radio_.next_msg_time) {
            bool success = send_location_msg(now);

            if (success) {
                radio_.next_msg_time = etl::nullopt;
            } else {
                radio_.next_msg_time = now + RADIO_RETRY_DELAY;
            }
        }

        if (movement) {
            to_tracking(now);
        } else {
            sleep(now);
        }
    }

    void to_tracking(uint32_t now)
    {
        logger::info("Entering live tracking state");

        state_ = state_t::TRACKING;

        gps_.instance.wake_up();
        gps_.next_probe_time = now;
        gps_.idle_probes.clear();
        gps_.n_idle = 0;

        if (radio_.next_msg_time.has_value()) {
            radio_.next_msg_time = min(*radio_.next_msg_time, now + TRACKING_RADIO_DELAY);
        } else {
            radio_.next_msg_time = now + TRACKING_RADIO_DELAY;
        }

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

    // Sleep into a low power sleep mode until the next GPS or radio event.
    //
    // Wake up on movement detection if movement detector is enabled.
    void sleep(uint32_t now)
    {
        uint32_t next_event = gps_.next_probe_time;
        if (radio_.next_msg_time.has_value()) {
            next_event = min(next_event, *radio_.next_msg_time);
        }

        unsigned long duration = max(500, (next_event - now) * 1000); // min 500ms

        logger::info("Sleep for " + String(duration) + " ms");
        led_t::blue.off();

        if (!DEBUG) {
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
                float dist = gps_t::distance(position.coordinates, gps_.last_position.coordinates);

                float speed = dist / delta_secs;

                float alt_gain;
                {
                    float smoothed_alt = 
                        gps_.smoothed_alt * (1.0f - GPS_ALT_SMOOTHER_FACTOR) +
                        position.coordinates.alt * GPS_ALT_SMOOTHER_FACTOR;

                    alt_gain =
                        smoothed_alt > gps_.smoothed_alt ?
                        smoothed_alt - gps_.smoothed_alt :
                        0;
                }

                bool is_idle;
                {
                    // Ignore altitude changes when veryfing if the device moves.
                    float horiz_dist = gps_t::distance(
                        position.coordinates, gps_.last_position.coordinates, true);
                    float horiz_speed = horiz_dist / delta_secs;

                    is_idle = horiz_speed < IDLE_THRESHOLD;
                }

                logger::info(
                    "\tDistance: " + String(dist, 2) + "m - " +
                    "Speed: " + String(speed, 2) + "m/s  - " +
                    "Alt. gain: " + String(alt_gain, 2) + "m - " +
                    "Idle: " + String(is_idle));

                if (is_idle) {
                    result = probe_result_t::IDLE;
                } else {
                    gps_.distance += dist;
                    gps_.alt_gain += alt_gain;

                    if (speed > gps_.max_speed) {
                        gps_.max_speed = speed;
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

    bool send_location_msg(uint64_t now)
    {
        logger::info("Send location message");

        float lat, lng, alt;
        if (gps_.has_position && gps_.last_position_time > radio_.last_msg_time) {
            lat = gps_.last_position.coordinates.lat;
            lng = gps_.last_position.coordinates.lng;
            alt = gps_.last_position.coordinates.alt;
        } else {
            logger::warning("\tNo new location update.");
            lat = lng = alt = 0.0f;
        }

        uint32_t last_msg = now - radio_.last_msg_time;

        radio_t::location_msg_t msg{lat, lng, alt, gps_.distance, gps_.alt_gain, gps_.max_speed};

        etl::optional<uint64_t> response = radio_.instance.send(msg);

        if (response.has_value()) {
            gps_.distance = 0.0f;
            gps_.alt_gain = 0.0f;
            gps_.max_speed = 0.0f;

            radio_.last_msg_time = now;
        }

        return response.has_value();
    }
};

} // namespace bike_tracker
