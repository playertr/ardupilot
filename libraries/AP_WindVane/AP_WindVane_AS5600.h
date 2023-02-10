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

class AP_WindVane_AS5600 : public AP_WindVane_Backend
{
    
public:
    // constructor
    using AP_WindVane_Backend::AP_WindVane_Backend;
    ~AP_WindVane_AS5600(void) {}

    // initialization
    void init() override;

    // update state
    void update_direction() override;

    void calibrate() override;

private:

    uint16_t readRawAngle();

    bool setPowerMode(uint8_t powerMode);

    uint16_t _raw_angle_offset = 0;

    // latest values read in
    float _wind_dir_rad;

    // pointer to I2C device
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev = nullptr;
};
