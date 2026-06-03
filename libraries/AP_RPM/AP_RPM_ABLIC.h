#pragma once

#include "AP_RPM.h"
#include "RPM_Backend.h"

#if AP_RPM_ENABLED

class AP_RPM_ABLIC : public AP_RPM_Backend
{
public:
    // constructor
    AP_RPM_ABLIC(AP_RPM &comp, uint8_t instance, AP_RPM::RPM_State &_state);

    // update state
    void update(void) override;

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    // ABLIC S-35770 specific values
    static const uint8_t I2C_ADDR = 0x32; // 0110010b = 0x32

    // Timer to handle 1s polling
    uint32_t _last_read_ms;

    // Timer to run I2C operations on background thread
    void _timer();
};

#endif // AP_RPM_ENABLED
