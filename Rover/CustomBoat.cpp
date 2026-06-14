#include "Rover.h"
#include "CustomBoat.h"
#include <GCS_MAVLink/GCS.h>

CustomBoat custom_boat;

extern const AP_HAL::HAL& hal;

CustomBoat::CustomBoat() :
    _trim_angle(-999.0f),
    _trim_pwm(1500),
    _battery_voltage(-999.0f),
    _rudder_angle(-999.0f),
    _fuel_level(-999.0f),
    _lights_status(-999.0f),
    _trim_status(-999.0f),
    _telem_step(0),
    _last_telem_ms(0),
    _trim_cmd(0),
    _last_trim_cmd_ms(0)
{
}

void CustomBoat::init()
{
    // Initialize I2C devices on Bus 1 safely after HAL boot
    _dev_ads1115 = std::move(hal.i2c_mgr->get_device(1, 0x48));
    _dev_tca9534_a = std::move(hal.i2c_mgr->get_device(1, 0x21));
    _dev_tca9534_b = std::move(hal.i2c_mgr->get_device(1, 0x20));

    if (_dev_tca9534_a && _dev_tca9534_a->get_semaphore()->take(10)) {
        uint8_t config_a[2] = {0x03, 0x0F};
        _dev_tca9534_a->transfer(config_a, 2, nullptr, 0);
        _dev_tca9534_a->get_semaphore()->give();
    }

    if (_dev_tca9534_b && _dev_tca9534_b->get_semaphore()->take(10)) {
        uint8_t config_b[2] = {0x03, 0xFF};
        _dev_tca9534_b->transfer(config_b, 2, nullptr, 0);
        _dev_tca9534_b->get_semaphore()->give();
    }

    if (_dev_ads1115) {
        _dev_ads1115->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&CustomBoat::_timer, void));
    } else if (_dev_tca9534_a) {
        _dev_tca9534_a->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&CustomBoat::_timer, void));
    } else if (_dev_tca9534_b) {
        _dev_tca9534_b->register_periodic_callback(20000, FUNCTOR_BIND_MEMBER(&CustomBoat::_timer, void));
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
                case 0: _trim_angle = (raw_val * rover.g.cust_trim_mult.get()) + rover.g.cust_trim_off.get(); break;
                case 1: _battery_voltage = (raw_val * rover.g.cust_bat_mult.get()) + rover.g.cust_bat_off.get(); break;
                case 2: _rudder_angle = (raw_val * rover.g.cust_rudd_mult.get()) + rover.g.cust_rudd_off.get(); break;
                case 3: _fuel_level = (raw_val * rover.g.cust_fuel_mult.get()) + rover.g.cust_fuel_off.get(); break;
            }
        } else {
            switch (ads_ch) {
                case 0: _trim_angle = -999.0f; break;
                case 1: _battery_voltage = -999.0f; break;
                case 2: _rudder_angle = -999.0f; break;
                case 3: _fuel_level = -999.0f; break;
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
    } else {
        _lights_status = -999.0f;
    }
}

void CustomBoat::handle_trim()
{
    if (!_dev_tca9534_a) {
        return;
    }

    // 1. Determine Trim Commands (From MAV_CMD_USER_1 with 1s timeout)
    uint8_t out_val = 0x00; // default both off

    if (AP_HAL::millis() - _last_trim_cmd_ms > 1000) {
        _trim_cmd = 0; // Timeout, revert to NONE
    }

    if (_trim_cmd == 1) {
        // TRIM_UP: Output 1 on Ch4, 0 on Ch5. Wait, the prompt says "trim up output is TCA9534 address 001 ch4. trim down output is TCA9534 address 001 ch5."
        // Earlier the prompt said RC 8 controlled ch0 and ch1 as outputs and ch4/ch5 were inputs!
        // The user says: "trim up output is TCA9534 address 001 ch4. trim down output is TCA9534 address 001 ch5. Trim up/down status reading is TCA9534 address 001 ch0-1."
        // Oh! They swapped the physical pins from the first document. I need to update the direction register as well!
        out_val |= 0x10; // Ch 4 High
    } else if (_trim_cmd == 2) {
        // TRIM_DOWN
        out_val |= 0x20; // Ch 5 High
    }

    // Write to Output Port Register (0x01)
    uint8_t write_buf[2] = {0x01, out_val};
    _dev_tca9534_a->transfer(write_buf, 2, nullptr, 0);

    // 2. Read Trim Status (Ch 4 and 5)
    uint8_t reg = 0x00;
    uint8_t rx_buf[1];
    if (_dev_tca9534_a->transfer(&reg, 1, rx_buf, 1)) {
        _trim_status = rx_buf[0] & 0x03; // read Ch0 and Ch1 directly
    } else {
        _trim_status = -999.0f;
    }
}

void CustomBoat::update()
{
    // Safely read RC input on the main scheduler thread
    _trim_pwm = hal.rcin->read(7);

    // Check if it is time to send the next telemetry variable
    uint32_t now = AP_HAL::millis();
    uint16_t delay_ms = rover.g.cust_tlm_dely.get();

    if ((now - _last_telem_ms) >= delay_ms) {
        _last_telem_ms = now;

        float rpm_val = 0.0f;
#if AP_RPM_ENABLED
        auto *rpm = AP::rpm();
        if (rpm) {
            rpm->get_rpm(0, rpm_val); // Get instance 0 RPM
        }
#endif

        switch (_telem_step) {
            case 0:
                gcs().send_named_float("TRIM_ANG", _trim_angle);
                break;
            case 1:
                gcs().send_named_float("RUDD_ANG", _rudder_angle);
                break;
            case 2:
                gcs().send_named_float("BAT_VOLT", _battery_voltage);
                break;
            case 3:
                gcs().send_named_float("FUEL_LVL", _fuel_level);
                break;
            case 4:
                gcs().send_named_float("LGT_STAT", _lights_status);
                break;
            case 5:
                gcs().send_named_float("TRM_STAT", _trim_status);
                break;
            case 6:
                gcs().send_named_float("ENG_RPM", rpm_val);
                break;
            case 7:
                gcs().send_named_float("DUMMY_VAR", 0.0f); // 8th variable as requested
                break;
        }

        _telem_step++;
        if (_telem_step > 7) {
            _telem_step = 0;
        }
    }
}


void CustomBoat::set_trim_command(uint8_t cmd)
{
    _trim_cmd = cmd;
    _last_trim_cmd_ms = AP_HAL::millis();
}
