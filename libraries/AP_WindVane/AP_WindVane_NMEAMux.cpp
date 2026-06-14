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

#include "AP_WindVane_NMEAMux.h"
#include <AP_NMEAMux/AP_NMEAMux.h>

#if AP_WINDVANE_ENABLED

void AP_WindVane_NMEAMux::update_speed()
{
    auto *nmeamux = AP::nmeamux();
    if (nmeamux == nullptr) {
        return;
    }

    float speed;
    if (nmeamux->get_wind_speed(speed)) {
        _frontend._speed_apparent_raw = speed;
    }
}

void AP_WindVane_NMEAMux::update_direction()
{
    auto *nmeamux = AP::nmeamux();
    if (nmeamux == nullptr) {
        return;
    }

    float angle;
    if (nmeamux->get_wind_angle(angle)) {
        _frontend._direction_apparent_raw = wrap_PI(radians(angle));
    }
}

#endif  // AP_WINDVANE_ENABLED
