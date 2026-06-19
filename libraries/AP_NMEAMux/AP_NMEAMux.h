/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Math/AP_Math.h>

class AP_NMEAMux {
public:
    AP_NMEAMux();

    // Singleton pattern
    static AP_NMEAMux *get_singleton(void);

    // Initialise the singleton
    void init(void);

    // Update the NMEA parser, expected to be called at 1Hz or faster
    void update(void);

    // Data getters
    // Wind data (MWV, MWD)
    bool get_wind_speed(float &speed) const;
    bool get_wind_direction(float &direction) const; // true direction
    bool get_wind_angle(float &angle) const;

    // Environmental data (MDA, XDR)
    bool get_air_temperature(float &temperature) const;
    bool get_barometric_pressure(float &pressure) const;

    // Water depth (DPT, DBT)
    bool get_water_depth(float &depth) const;

    // Speed through water (VHW)
    bool get_speed_through_water(float &speed) const;

private:
    static AP_NMEAMux *_singleton;

    AP_HAL::UARTDriver *_uart;

    // Buffer for reading from UART
    static const uint16_t MAX_NMEA_LENGTH = 120;
    char _buffer[MAX_NMEA_LENGTH];
    uint16_t _buffer_pos;

    // Parsing helpers
    void process_sentence(void);
    bool check_checksum(const char *sentence);
    bool decode_mwv(const char *sentence);
    bool decode_mwd(const char *sentence);
    bool decode_mda(const char *sentence);
    bool decode_xdr(const char *sentence);
    bool decode_dpt(const char *sentence);
    bool decode_dbt(const char *sentence);
    bool decode_vhw(const char *sentence);

    // Internal data structures
    struct Data {
        float value;
        uint32_t last_update_ms; // 1Hz throttle check
        uint32_t last_receive_ms; // 3000ms timeout check
    };

    Data _wind_speed;
    Data _wind_direction;
    Data _wind_angle;
    Data _air_temperature;
    Data _barometric_pressure;
    Data _water_depth;
    Data _speed_through_water;

    // Throttle / Timeout constants
    static const uint32_t THROTTLE_MS = 1000;
    static const uint32_t TIMEOUT_MS = 3000;

    void update_data(Data &data, float new_value);
    bool is_data_valid(const Data &data) const;
};

namespace AP {
    AP_NMEAMux *nmeamux();
}
