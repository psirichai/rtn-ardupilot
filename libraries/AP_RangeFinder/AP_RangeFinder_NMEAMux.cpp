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

#include "AP_RangeFinder_NMEAMux.h"
#include <AP_NMEAMux/AP_NMEAMux.h>

#if AP_RANGEFINDER_ENABLED

void AP_RangeFinder_NMEAMux::update(void)
{
    auto *nmeamux = AP::nmeamux();
    if (nmeamux == nullptr) {
        set_status(RangeFinder::Status::NoData);
        return;
    }

    float depth;
    if (nmeamux->get_water_depth(depth)) {
        state.distance_m = depth;
        update_status();
    } else {
        set_status(RangeFinder::Status::NoData);
    }
}

#endif  // AP_RANGEFINDER_ENABLED
