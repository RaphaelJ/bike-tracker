#pragma once

#include <cmath>
#include <cstdint>

#include <Arduino.h>
#include <SigFox.h>
#include <etl/optional.h>

#include "logger.hpp"

namespace bike_tracker {

class radio_t {
public:

    struct location_msg_t {
        static_assert(sizeof(float) == 4);
        
        // Latest location. Has a 0 value if no valid location.
        float lat;
        float lng;

        // Latest altitude, in meters, divided by 8 (range: [0..2040] m).
        // Do not go to the Netherlands or the Alps.
        // Has a 0 value if no valid location.
        uint8_t alt; 

        // Distance since the last message, in meters per divided by 16 (range: [0..4080] m).
        uint8_t dist;

        // Positive elevation gain, in meters divided by 2 (range: [0..510] m).
        uint8_t alt_gain;

        // Maximum speed, in meters per second divided by 16 (range between [0..57.375] kph).
        uint8_t max_speed;

        // Constructs the message with the actual, non scaled, values.
        location_msg_t(
            float lat_, float lng_, float alt_,
            float dist_, float alt_gain_, float max_speed_) :
            lat(lat_), lng(lng_), alt(round(alt_ / 8)),
            dist(round(dist_ / 16)),
            alt_gain(round(alt_gain_ / 2)),
            max_speed(round(max_speed_ / 16))
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

    // Sends the given message.
    //
    // On succes, returns a 8 byte callback response.
    template<typename msg_t>
    etl::optional<uint64_t>
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

        bool status = SigFox.endPacket(true);

        etl::optional<uint64_t> response;

        if (status != 0) {
            logger::warning(
                "Error while transmitting SigFox paquet (status: 0x" + String(status, HEX) + ")");
        } else {    
            uint64_t value{0};
            String value_str = String();

            while (SigFox.available()) {
                byte byte_val = (byte) SigFox.read();

                value <<= 8;
                value |= byte_val;

                value_str += String(byte_val, HEX);
            }

            logger::info("Received callback response: 0x" + value_str);

            response.emplace(value);
        }

        SigFox.noDebug();
        sleep();

        return response;
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