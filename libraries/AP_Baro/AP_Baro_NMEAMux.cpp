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

#include "AP_Baro_NMEAMux.h"
#include <AP_NMEAMux/AP_NMEAMux.h>

extern const AP_HAL::HAL& hal;

AP_Baro_NMEAMux::AP_Baro_NMEAMux(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
    set_bus_id(_instance, AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_SERIAL, 0, 0, DEVTYPE_BARO_NMEAMUX));
}

void AP_Baro_NMEAMux::update()
{
    auto *nmeamux = AP::nmeamux();
    if (nmeamux == nullptr) {
        return;
    }

    float pressure, temperature;
    if (nmeamux->get_barometric_pressure(pressure) && nmeamux->get_air_temperature(temperature)) {
        _copy_to_frontend(_instance, pressure, temperature);
    }
}
