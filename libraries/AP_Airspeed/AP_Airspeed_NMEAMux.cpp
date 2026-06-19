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

#include "AP_Airspeed_NMEAMux.h"
#include <AP_NMEAMux/AP_NMEAMux.h>

#if AP_AIRSPEED_ENABLED

bool AP_Airspeed_NMEAMux::init()
{
    return true;
}

bool AP_Airspeed_NMEAMux::get_airspeed(float &airspeed)
{
    auto *nmeamux = AP::nmeamux();
    if (nmeamux == nullptr) {
        return false;
    }

    return nmeamux->get_speed_through_water(airspeed);
}

bool AP_Airspeed_NMEAMux::get_temperature(float &temperature)
{
    auto *nmeamux = AP::nmeamux();
    if (nmeamux == nullptr) {
        return false;
    }

    return nmeamux->get_air_temperature(temperature);
}

#endif  // AP_AIRSPEED_ENABLED
