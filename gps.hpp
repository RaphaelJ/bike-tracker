#pragma once

#include <Arduino.h>
#include <cmath>

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include "logger.hpp"

namespace bike_tracker {

class gps_t {
public:
    struct date_time_t {
        bool has_date;

        int16_t year;
        uint8_t month;
        uint8_t day;

        bool has_time;

        uint8_t hour;
        uint8_t minute;
        uint8_t second;
    };

    struct coordinates_t {
        float lat; // in degrees, [+90..-90]
        float lng; // in degrees, [+180..-180]
        float alt; // above sea level in meters
    };

    struct position_t {
        bool has_gnss_fix;
        uint8_t n_satellites;

        coordinates_t coordinates;

        date_time_t date_time;
    };

    gps_t(Uart &serial = Serial1) : serial_(serial)
    {
    }

    void
    setup() 
    {
        serial_.begin(9600);
        while(!serial_);

        if (!instance_.begin(serial_)) {
            logger::error("Unable to setup GPS.");
        }

        for (const gnss_config_t &gnss : GNSS_CONFIG) {
            instance_.enableGNSS(gnss.enabled, gnss.id);
        }

        powerSave(false);

        logger::info("GPS successfuly initialized.");
    }

    position_t get_position()
    {
        position_t pos;

        pos.has_gnss_fix = instance_.getGnssFixOk();
        pos.n_satellites = instance_.getSIV();

        if (pos.has_gnss_fix) {
            pos.coordinates.lat = ((float) instance_.getLatitude()) * 0.0000001f;
            pos.coordinates.lng = ((float) instance_.getLongitude()) * 0.0000001f;
            pos.coordinates.alt = ((float) instance_.getAltitudeMSL()) * 0.001f;
        }

        pos.date_time.has_date = instance_.getDateValid();
        if (pos.date_time.has_date) {
            pos.date_time.year = instance_.getYear();
            pos.date_time.month = instance_.getMonth();
            pos.date_time.day = instance_.getDay();
        }

        pos.date_time.has_time = instance_.getTimeValid();
        if (pos.date_time.has_time) {
            pos.date_time.hour = instance_.getHour();
            pos.date_time.minute = instance_.getMinute();
            pos.date_time.second = instance_.getSecond();
        }

        return pos;
    }

    const SFE_UBLOX_GNSS &instance()
    {
        return instance_;
    }

    bool powerSave()
    {
        return instance_.getPowerSaveMode();
    }

    void powerSave(bool enabled)
    {
        instance_.powerSaveMode(enabled);   
    }

    // Computes the distance (in meters) between two coordinates.
    static float distance(const coordinates_t &coord_a, const coordinates_t &coord_b)
    {   
        // Based on http://www.movable-type.co.uk/scripts/latlong.html.

        constexpr float earth_raduis = 6371.0f * 1000.0f; // in meters

        auto to_radians = [](float value) {
            return value * M_PI / 180.0f;
        };

        float lat_a = to_radians(coord_a.lat);
        float lat_b = to_radians(coord_b.lat);

        float delta_lat = to_radians(coord_a.lat - coord_b.lat);
        float delta_lng = to_radians(coord_a.lng - coord_b.lng);

        float a = pow(sin(delta_lat  / 2), 2)
                + cos(lat_a) * cos(lat_b) * pow(sin(delta_lng / 2), 2);
        float c = 2 * atan2(sqrt(a), sqrt(1 - a));

        float horiz_dist = c * earth_raduis;

        float vert_dist = abs(coord_a.alt - coord_b.alt);

        return sqrt(pow(horiz_dist, 2) + pow(vert_dist, 2));
    }

private:
    struct gnss_config_t {
        sfe_ublox_gnss_ids_e id;
        bool enabled;
    };

    static const gnss_config_t GNSS_CONFIG[7];

    Uart &serial_;

    SFE_UBLOX_GNSS instance_;
};

const gps_t::gnss_config_t gps_t::GNSS_CONFIG[] = {
    {SFE_UBLOX_GNSS_ID_GPS, true},
    {SFE_UBLOX_GNSS_ID_SBAS, false},
    {SFE_UBLOX_GNSS_ID_GALILEO, true},
    {SFE_UBLOX_GNSS_ID_BEIDOU, false},
    {SFE_UBLOX_GNSS_ID_IMES, false},
    {SFE_UBLOX_GNSS_ID_QZSS, false},
    {SFE_UBLOX_GNSS_ID_GLONASS, true}
};

}