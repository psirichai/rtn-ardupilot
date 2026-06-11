#include "Rover.h"
#include "CustomBoat.h"
#include <GCS_MAVLink/GCS.h>

CustomBoat custom_boat;

extern const AP_HAL::HAL& hal;

CustomBoat::CustomBoat() :
    _trim_angle(0.0f),
    _battery_voltage(0.0f),
    _rudder_angle(0.0f),
    _fuel_level(0.0f),
    _lights_status(0),
    _trim_status(0)
{
}

void CustomBoat::init()
{
    // Initialize I2C devices on Bus 1 safely after HAL boot
    _dev_ads1115 = std::move(hal.i2c_mgr->get_device(1, 0x48));
    _dev_tca9534_a = std::move(hal.i2c_mgr->get_device(1, 0x39));
    _dev_tca9534_b = std::move(hal.i2c_mgr->get_device(1, 0x38));

    if (_dev_tca9534_a) {
        uint8_t config_a[2] = {0x03, 0xFC};
        _dev_tca9534_a->transfer(config_a, 2, nullptr, 0);
    }

    if (_dev_tca9534_b) {
        uint8_t config_b[2] = {0x03, 0xFF};
        _dev_tca9534_b->transfer(config_b, 2, nullptr, 0);
    }

    if (_dev_ads1115) {
        _dev_ads1115->register_periodic_callback(200000, FUNCTOR_BIND_MEMBER(&CustomBoat::_timer, void));
    }
}

// State machine for ADS1115 non-blocking reads
static uint8_t ads_ch = 0;
static bool waiting_conversion = false;

void CustomBoat::read_ads1115()
{
    if (!_dev_ads1115) {
        return;
    }

    if (!waiting_conversion) {
        // Start conversion
        uint16_t config = 0x8183; // OS=1, PGA=000, MODE=1, DR=100
        config |= ((4 + ads_ch) << 12); // MUX

        uint8_t config_buf[3] = {0x01, (uint8_t)(config >> 8), (uint8_t)(config & 0xFF)};
        _dev_ads1115->transfer(config_buf, 3, nullptr, 0);
        waiting_conversion = true;
    } else {
        // Read conversion
        uint8_t reg = 0x00;
        uint8_t rx_buf[2];
        if (_dev_ads1115->transfer(&reg, 1, rx_buf, 2)) {
            int16_t raw_val = (rx_buf[0] << 8) | rx_buf[1];

            switch (ads_ch) {
                case 0:
                    _trim_angle = (raw_val * rover.g.cust_trim_mult.get()) + rover.g.cust_trim_off.get();
                    break;
                case 1:
                    _battery_voltage = (raw_val * rover.g.cust_bat_mult.get()) + rover.g.cust_bat_off.get();
                    break;
                case 2:
                    _rudder_angle = (raw_val * rover.g.cust_rudd_mult.get()) + rover.g.cust_rudd_off.get();
                    break;
                case 3:
                    _fuel_level = (raw_val * rover.g.cust_fuel_mult.get()) + rover.g.cust_fuel_off.get();
                    break;
            }
        }

        // Move to next channel
        ads_ch = (ads_ch + 1) % 4;
        waiting_conversion = false;
    }
}

void CustomBoat::_timer()
{
    read_ads1115();
    read_lights_status();
    handle_trim();
}


void CustomBoat::read_lights_status()
{
    if (!_dev_tca9534_b) {
        return;
    }

    // Read Input Port Register (0x00)
    uint8_t reg = 0x00;
    uint8_t rx_buf[1];
    if (_dev_tca9534_b->transfer(&reg, 1, rx_buf, 1)) {
        _lights_status = rx_buf[0];
    }
}

void CustomBoat::handle_trim()
{
    if (!_dev_tca9534_a) {
        return;
    }

    // 1. Read RC Channel 8 for Trim Commands
    // 1000-1400: Trim Down, 1400-1600: Neutral, 1600-2000: Trim Up
    uint16_t ch8_pwm = hal.rcin->read(7); // 0-indexed

    uint8_t out_val = 0x00; // default both off (assuming active high for relays)

    if (ch8_pwm > 1600) {
        out_val |= 0x01; // Ch 0 High (Trim Up)
    } else if (ch8_pwm < 1400 && ch8_pwm > 900) {
        out_val |= 0x02; // Ch 1 High (Trim Down)
    }

    // Write to Output Port Register (0x01)
    uint8_t write_buf[2] = {0x01, out_val};
    _dev_tca9534_a->transfer(write_buf, 2, nullptr, 0);

    // 2. Read Trim Status (Ch 4 and 5)
    uint8_t reg = 0x00;
    uint8_t rx_buf[1];
    if (_dev_tca9534_a->transfer(&reg, 1, rx_buf, 1)) {
        _trim_status = (rx_buf[0] >> 4) & 0x03; // shift to get Ch4, Ch5 at bit 0, 1
    }
}

void CustomBoat::update()
{
    // Broadcast MAVLink Telemetry
    gcs().send_named_float("TRIM_ANG", _trim_angle);
    gcs().send_named_float("RUDD_ANG", _rudder_angle);
    gcs().send_named_float("BAT_VOLT", _battery_voltage);
    gcs().send_named_float("FUEL_LVL", _fuel_level);
    gcs().send_named_float("LGT_STAT", (float)_lights_status);
    gcs().send_named_float("TRM_STAT", (float)_trim_status);
}
