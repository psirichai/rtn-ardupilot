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

#include "AP_NMEAMux.h"
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

AP_NMEAMux *AP_NMEAMux::_singleton;

AP_NMEAMux::AP_NMEAMux() :
    _uart(nullptr),
    _buffer_pos(0)
{
    _singleton = this;
}

AP_NMEAMux *AP_NMEAMux::get_singleton(void)
{
    return _singleton;
}

void AP_NMEAMux::init(void)
{
    _uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_NMEAMux, 0);
    if (_uart != nullptr) {
        _uart->begin(AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_NMEAMux, 0));
    }
}

void AP_NMEAMux::update(void)
{
    if (_uart == nullptr) {
        return;
    }

    uint32_t bytes_available = _uart->available();
    while (bytes_available-- > 0) {
        int16_t b = _uart->read();
        if (b < 0) {
            break;
        }

        char c = (char)b;

        if (c == '\n' || c == '\r') {
            if (_buffer_pos > 0) {
                _buffer[_buffer_pos] = '\0';
                process_sentence();
                _buffer_pos = 0;
            }
        } else if (_buffer_pos < MAX_NMEA_LENGTH - 1) {
            _buffer[_buffer_pos++] = c;
        } else {
            // Buffer overflow, drop it
            _buffer_pos = 0;
        }
    }
}

bool AP_NMEAMux::check_checksum(const char *sentence)
{
    if (sentence[0] != '$') {
        return false;
    }

    const char *asterisk = strchr(sentence, '*');
    if (asterisk == nullptr) {
        return false;
    }

    uint8_t calculated_checksum = 0;
    for (const char *p = sentence + 1; p < asterisk; p++) {
        calculated_checksum ^= *p;
    }

    uint8_t provided_checksum = (uint8_t)strtol(asterisk + 1, nullptr, 16);

    return calculated_checksum == provided_checksum;
}

static const char *get_term(const char *sentence, uint8_t index)
{
    const char *p = sentence;
    for (uint8_t i = 0; i < index; i++) {
        p = strchr(p, ',');
        if (p == nullptr) {
            return nullptr;
        }
        p++;
    }
    return p;
}

void AP_NMEAMux::process_sentence(void)
{
    if (!check_checksum(_buffer)) {
        return;
    }

    // copy buffer for tokenizing
    char temp_buffer[MAX_NMEA_LENGTH];
    strncpy(temp_buffer, _buffer, MAX_NMEA_LENGTH - 1);
    temp_buffer[MAX_NMEA_LENGTH - 1] = '\0';

    const char *type = get_term(temp_buffer, 0);
    if (type == nullptr || strlen(type) < 5) {
        return;
    }

    const char *sentence_type = type + 3; // Skip talker ID, e.g., "WI" from "$WIMWV"

    // Black-listed Sentences
    if (strncmp(sentence_type, "MTW", 3) == 0 ||
        strncmp(sentence_type, "GGA", 3) == 0 ||
        strncmp(sentence_type, "RMC", 3) == 0 ||
        strncmp(sentence_type, "ZDA", 3) == 0 ||
        strncmp(sentence_type, "HDG", 3) == 0 ||
        strncmp(sentence_type, "HDT", 3) == 0 ||
        strncmp(sentence_type, "ROT", 3) == 0) {
        return;
    }

    // White-listed Sentences
    if (strncmp(sentence_type, "MWV", 3) == 0) {
        decode_mwv(temp_buffer);
    } else if (strncmp(sentence_type, "MWD", 3) == 0) {
        decode_mwd(temp_buffer);
    } else if (strncmp(sentence_type, "MDA", 3) == 0) {
        decode_mda(temp_buffer);
    } else if (strncmp(sentence_type, "XDR", 3) == 0) {
        decode_xdr(temp_buffer);
    } else if (strncmp(sentence_type, "DPT", 3) == 0) {
        decode_dpt(temp_buffer);
    } else if (strncmp(sentence_type, "DBT", 3) == 0) {
        decode_dbt(temp_buffer);
    } else if (strncmp(sentence_type, "VHW", 3) == 0) {
        decode_vhw(temp_buffer);
    }
}

// $WIMWV,angle,reference,speed,speed_units,status*cs
bool AP_NMEAMux::decode_mwv(const char *sentence)
{
    char temp_sentence[MAX_NMEA_LENGTH];
    strncpy(temp_sentence, sentence, MAX_NMEA_LENGTH - 1);

    const char *angle_str = get_term(temp_sentence, 1);
    const char *ref_str = get_term(temp_sentence, 2);
    const char *speed_str = get_term(temp_sentence, 3);
    const char *unit_str = get_term(temp_sentence, 4);
    const char *status_str = get_term(temp_sentence, 5);

    if (!angle_str || !ref_str || !speed_str || !unit_str || !status_str) return false;
    if (status_str[0] != 'A') return false; // Invalid data

    float speed = strtof(speed_str, nullptr);
    if (unit_str[0] == 'N') speed *= 0.514444f; // Knots to m/s
    else if (unit_str[0] == 'K') speed *= 0.277778f; // km/h to m/s

    float angle = strtof(angle_str, nullptr);

    update_data(_wind_speed, speed);

    if (ref_str[0] == 'R') { // Relative/Apparent
        update_data(_wind_angle, angle);
    } else if (ref_str[0] == 'T') { // True
        update_data(_wind_direction, angle);
    }

    return true;
}

// $WIMWD,direction_T,T,direction_M,M,speed_N,N,speed_M,M*cs
bool AP_NMEAMux::decode_mwd(const char *sentence)
{
    char temp_sentence[MAX_NMEA_LENGTH];
    strncpy(temp_sentence, sentence, MAX_NMEA_LENGTH - 1);

    const char *dir_t_str = get_term(temp_sentence, 1);
    const char *speed_m_str = get_term(temp_sentence, 7);

    if (dir_t_str && strlen(dir_t_str) > 0) {
        update_data(_wind_direction, strtof(dir_t_str, nullptr));
    }
    if (speed_m_str && strlen(speed_m_str) > 0) {
        update_data(_wind_speed, strtof(speed_m_str, nullptr));
    }

    return true;
}

// $WIMDA,pressure_inHg,I,pressure_bar,B,air_temp_C,C,...
bool AP_NMEAMux::decode_mda(const char *sentence)
{
    char temp_sentence[MAX_NMEA_LENGTH];
    strncpy(temp_sentence, sentence, MAX_NMEA_LENGTH - 1);

    const char *press_bar_str = get_term(temp_sentence, 3);
    const char *temp_c_str = get_term(temp_sentence, 5);

    if (press_bar_str && strlen(press_bar_str) > 0) {
        // convert bar to Pascals
        update_data(_barometric_pressure, strtof(press_bar_str, nullptr) * 100000.0f);
    }
    if (temp_c_str && strlen(temp_c_str) > 0) {
        update_data(_air_temperature, strtof(temp_c_str, nullptr));
    }

    return true;
}

// $WIXDR,type,data,unit,id,...
bool AP_NMEAMux::decode_xdr(const char *sentence)
{
    char temp_sentence[MAX_NMEA_LENGTH];
    strncpy(temp_sentence, sentence, MAX_NMEA_LENGTH - 1);

    // XDR can have multiple datasets in one sentence
    uint8_t index = 1;
    while (true) {
        const char *type_str = get_term(temp_sentence, index);
        const char *data_str = get_term(temp_sentence, index + 1);
        const char *unit_str = get_term(temp_sentence, index + 2);

        if (!type_str || !data_str || !unit_str) break;

        if (type_str[0] == 'P' && unit_str[0] == 'B') {
            update_data(_barometric_pressure, strtof(data_str, nullptr) * 100000.0f); // bar to Pa
        } else if (type_str[0] == 'P' && unit_str[0] == 'P') {
            update_data(_barometric_pressure, strtof(data_str, nullptr)); // Pascal
        } else if (type_str[0] == 'C' && unit_str[0] == 'C') {
            update_data(_air_temperature, strtof(data_str, nullptr)); // Celcius
        }

        index += 4; // Move to next dataset

    }

    return true;
}

// $SDDPT,depth_meters,offset_meters,max_range*cs
bool AP_NMEAMux::decode_dpt(const char *sentence)
{
    char temp_sentence[MAX_NMEA_LENGTH];
    strncpy(temp_sentence, sentence, MAX_NMEA_LENGTH - 1);

    const char *depth_str = get_term(temp_sentence, 1);

    if (depth_str && strlen(depth_str) > 0) {
        float depth = strtof(depth_str, nullptr);
        update_data(_water_depth, depth);
    }
    return true;
}

// $SDDBT,feet,f,meters,M,fathoms,F*cs
bool AP_NMEAMux::decode_dbt(const char *sentence)
{
    char temp_sentence[MAX_NMEA_LENGTH];
    strncpy(temp_sentence, sentence, MAX_NMEA_LENGTH - 1);

    const char *depth_m_str = get_term(temp_sentence, 3);

    if (depth_m_str && strlen(depth_m_str) > 0) {
        update_data(_water_depth, strtof(depth_m_str, nullptr));
    }
    return true;
}

// $VDVHW,heading_T,T,heading_M,M,speed_N,N,speed_K,K*cs
bool AP_NMEAMux::decode_vhw(const char *sentence)
{
    char temp_sentence[MAX_NMEA_LENGTH];
    strncpy(temp_sentence, sentence, MAX_NMEA_LENGTH - 1);

    // Ignore heading fields as per requirement
    const char *speed_k_str = get_term(temp_sentence, 7); // Speed in km/h
    const char *speed_n_str = get_term(temp_sentence, 5); // Speed in knots

    float speed = 0;
    bool valid = false;

    if (speed_k_str && strlen(speed_k_str) > 0) {
        speed = strtof(speed_k_str, nullptr) * 0.277778f; // km/h to m/s
        valid = true;
    } else if (speed_n_str && strlen(speed_n_str) > 0) {
        speed = strtof(speed_n_str, nullptr) * 0.514444f; // knots to m/s
        valid = true;
    }

    if (valid) {
        update_data(_speed_through_water, speed);
    }

    return true;
}

void AP_NMEAMux::update_data(Data &data, float new_value)
{
    uint32_t now = AP_HAL::millis();

    // Throttle updates to 1Hz
    if (data.last_receive_ms == 0 || now - data.last_update_ms >= THROTTLE_MS) {
        data.value = new_value;
        data.last_update_ms = now;
    }
    data.last_receive_ms = now;
}

bool AP_NMEAMux::is_data_valid(const Data &data) const
{
    uint32_t now = AP_HAL::millis();
    return (now - data.last_receive_ms) < TIMEOUT_MS && data.last_receive_ms != 0;
}

bool AP_NMEAMux::get_wind_speed(float &speed) const
{
    if (is_data_valid(_wind_speed)) {
        speed = _wind_speed.value;
        return true;
    }
    return false;
}

bool AP_NMEAMux::get_wind_direction(float &direction) const
{
    if (is_data_valid(_wind_direction)) {
        direction = _wind_direction.value;
        return true;
    }
    return false;
}

bool AP_NMEAMux::get_wind_angle(float &angle) const
{
    if (is_data_valid(_wind_angle)) {
        angle = _wind_angle.value;
        return true;
    }
    return false;
}

bool AP_NMEAMux::get_air_temperature(float &temperature) const
{
    if (is_data_valid(_air_temperature)) {
        temperature = _air_temperature.value;
        return true;
    }
    return false;
}

bool AP_NMEAMux::get_barometric_pressure(float &pressure) const
{
    if (is_data_valid(_barometric_pressure)) {
        pressure = _barometric_pressure.value;
        return true;
    }
    return false;
}

bool AP_NMEAMux::get_water_depth(float &depth) const
{
    if (is_data_valid(_water_depth)) {
        depth = _water_depth.value;
        return true;
    }
    return false;
}

bool AP_NMEAMux::get_speed_through_water(float &speed) const
{
    if (is_data_valid(_speed_through_water)) {
        speed = _speed_through_water.value;
        return true;
    }
    return false;
}

namespace AP {
    AP_NMEAMux *nmeamux()
    {
        return AP_NMEAMux::get_singleton();
    }
}
