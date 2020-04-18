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

#include "AP_WindVane_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_WindVane_I2C : public AP_WindVane_Backend
{
public:
    // constructor
    AP_WindVane_I2C(AP_WindVane &frontend);
    ~AP_WindVane_I2C(void) {}

    // initialization
    void init() override;

    // update state
    void update_direction() override;
    void update_speed() override;

private:
    // See if we can read in some data
    void update();

    // cyclic redundancy check
    bool _crc(const uint8_t data[], unsigned size, uint8_t checksum);

    // latest values read in
    float _speed_ms;
    float _wind_dir_deg;

    HAL_Semaphore _sem;
    // pointer to I2C device
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev = nullptr;
};
