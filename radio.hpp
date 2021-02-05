#pragma once

#include <cmath>

#include <Arduino.h>
#include <SigFox.h>

#include "logger.hpp"

namespace bike_tracker {

class radio_t {
public:

    struct location_msg_t {
        static_assert(sizeof(float) == 4);
        
        // Latest location. Has a 0 value if no vlaid location.
        float lat;
        float lng;

        // Distance since the last message, in meters divided by 16 (range: [0..4080]).
        uint8_t dist;

        // Positive elevation gain, in meters divided by 2 (range: [0..510]).
        uint8_t alt_gain;

        // Maximum speed, in kph multiplied by 3 (range between [0..85] kph).
        uint8_t max_speed;  

        // The time since the last tansmitted location message, in seconds square-rooted
        // (range: [0..65025]).
        //
        // Has a 0 value if no previously transmitted message.
        uint8_t last_msg;

        /** Constructs the message with the actual, non scaled, values.
         * 
         * @param last_msg_ the delay since the last message in milliseconds.
         */
        location_msg_t(
            float lat_, float lng_, float dist_, float alt_gain_, float max_speed_,
            uint32_t last_msg_) :
            lat(lat_), lng(lng_),
            dist(round(dist_ / 16)),
            alt_gain(round(alt_gain_ / 2)),
            max_speed(round(max_speed_ * 3)),
            last_msg(round(sqrt(((float) last_msg_) / 1000.0f)))
        { }
    } __attribute__((packed));

    void
    setup() 
    {
        wake_up();

        logger::info(
            "\tAtm version: " + SigFox.AtmVersion() + " - " +
            "SigFox version: " + SigFox.SigVersion() + " " +
            "ID: " + SigFox.ID() + " " +
            "PA: " + SigFox.PAC() + " " +
            "Status: " + String(SigFox.statusCode(SIGFOX), HEX) + " - " +
            "Temp.: " + SigFox.internalTemperature() + "C°");

        sleep();
    }

    template<typename msg_t>
    bool
    send(const msg_t &msg)
    {
        static_assert(sizeof(msg) <= 12);

        logger::info("Sending " + String(sizeof(msg)) + " byte(s) message:");

        const byte *msg_bytes = reinterpret_cast<const byte *>(&msg);

        {
            String msg_hex;
            for (size_t i = 0; i < sizeof(msg); ++i) {
                msg_hex += String(msg_bytes[i], HEX) + " ";
            }
            logger::info("\t" + msg_hex);
        }

        wake_up();

        // Entering debug mode prevents a issue with the SigFox library, by disabling low power
        // optimisations.
        SigFox.debug();

        SigFox.beginPacket();

        SigFox.write(msg_bytes, sizeof(msg));

        bool success = SigFox.endPacket();

        if (!success) {
            logger::warning("Error while transmitting SigFox paquet.");
        }

        logger::info("SigFox status: " + String(SigFox.statusCode(SIGFOX), HEX));

        SigFox.noDebug();
        sleep();

        return success;
    }

    void 
    wake_up()
    {
        if (!SigFox.begin()) {
            logger::error("Unable to wake-up SigFox module.");
        }
        logger::info("SigFox module initialized.");
    }

    void
    sleep()
    {
        SigFox.end();
        logger::info("SigFox module now in deep sleep.");
    }

private:
};

}