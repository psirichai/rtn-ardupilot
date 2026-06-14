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
#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_ENABLED

#include "AP_Airspeed_Backend.h"

class AP_Airspeed_NMEAMux : public AP_Airspeed_Backend
{
public:

    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    // probe and initialise the sensor
    bool init(void) override;

    // this reads airspeed directly
    bool has_airspeed() override { return true; }

    // read the from the sensor
    bool get_airspeed(float &airspeed) override;

    // return the current temperature in degrees C
    bool get_temperature(float &temperature) override;
};

#endif  // AP_AIRSPEED_ENABLED
