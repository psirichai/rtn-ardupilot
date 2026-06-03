#pragma once

#include <AP_HAL/AP_HAL.h>

class CustomBoat
{
public:
    CustomBoat();

    // called at 1Hz from main loop
    void update();

    // background timer for I2C
    void _timer();

    // MAVLink broadcast is run on main thread
    void _broadcast_telemetry();


    // Telemetry getters
    float get_trim_angle() const { return _trim_angle; }
    float get_battery_voltage() const { return _battery_voltage; }
    float get_rudder_angle() const { return _rudder_angle; }
    float get_fuel_level() const { return _fuel_level; }

    uint8_t get_lights_status() const { return _lights_status; }
    uint8_t get_trim_status() const { return _trim_status; }

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_ads1115;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_tca9534_a; // Expander A (Trim)
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev_tca9534_b; // Expander B (Lights)

    float _trim_angle;
    float _battery_voltage;
    float _rudder_angle;
    float _fuel_level;

    uint8_t _lights_status;
    uint8_t _trim_status;

    // reads ADS1115 analog values and applies parameters
    void read_ads1115();

    // reads expander B (status of lights)
    void read_lights_status();

    // handles Trim Up/Down via RC channel 8 and expander A
    void handle_trim();
};

extern CustomBoat custom_boat;
