#include "AP_RPM_ABLIC.h"

#if AP_RPM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

extern const AP_HAL::HAL& hal;

/*
  constructor
 */
AP_RPM_ABLIC::AP_RPM_ABLIC(AP_RPM &comp, uint8_t instance, AP_RPM::RPM_State &_state) :
    AP_RPM_Backend(comp, instance, _state)
{
    // ABLIC S-35770 is typically on I2C bus 1 or 2 (we could use parameters to configure bus, using 1 here for default)
    _dev = std::move(hal.i2c_mgr->get_device(1, I2C_ADDR));
    if (!_dev) {
        return;
    }

    _dev->register_periodic_callback(1000000, FUNCTOR_BIND_MEMBER(&AP_RPM_ABLIC::_timer, void)); // 1Hz = 1000000 us
}

/*
  read and calculate RPM from sensor
 */
void AP_RPM_ABLIC::_timer(void)
{
    if (!_dev) {
        return;
    }

    uint8_t rx_buf[3]; // The S-35770 outputs up to 24 bits
    if (_dev->read_registers(0x00, rx_buf, sizeof(rx_buf))) {
        // Parse the read data (format based on typical ABLIC S-35770 response)
        // Assume basic pulse count over the 1 second interval
        uint32_t count = (rx_buf[0] << 16) | (rx_buf[1] << 8) | rx_buf[2];

        // Reset command (typically write a specific sequence or just writing 0 depending on precise datasheet)
        uint8_t reset_cmd = 0x00;
        _dev->transfer(&reset_cmd, 1, nullptr, 0);

        // Convert 1 second pulse count to RPM
        float rpm = count * 60.0f;

        state.rate_rpm = rpm;
        state.last_reading_ms = AP_HAL::millis();
        state.signal_quality = 1.0f;
    } else {
        state.rate_rpm = -999.0f;
        state.last_reading_ms = AP_HAL::millis();
        state.signal_quality = 0.0f;
    }
}

void AP_RPM_ABLIC::update(void)
{
    // handeled in background thread
}

#endif // AP_RPM_ENABLED
