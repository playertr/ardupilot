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

#include <AP_HAL/AP_HAL.h>
#include "AP_WindVane_I2C.h"
#include <AP_HAL/utility/OwnPtr.h>
#include <GCS_MAVLink/GCS.h>


#define INO_I2C_ADDRESS 0x08
#define INO_I2C_BUS 1

/*
    This driver queries an Arduino which is using I2C via the
    Wire library. 
*/

extern const AP_HAL::HAL& hal;

// constructor
AP_WindVane_I2C::AP_WindVane_I2C(AP_WindVane &frontend) :
    AP_WindVane_Backend(frontend)
{
    gcs().send_text(MAV_SEVERITY_INFO, "AP_WNDVN_I2C Constructed.");
}

// init - performs any required initialization for this instance
void AP_WindVane_I2C::init()
{
    const uint8_t address = INO_I2C_ADDRESS;

    _dev = hal.i2c_mgr->get_device(INO_I2C_BUS, address);
    if (!_dev) {
        hal.console->printf("No AP_WindVane_I2C found.");
        return;
    }

    hal.console->printf("AP_WindVane_I2C: Found on bus %u address 0x%02x",
                        INO_I2C_BUS, INO_I2C_ADDRESS);
    
    _dev->set_retries(2);
}

void AP_WindVane_I2C::update_direction()
{
    // Only call update from here if it has not been called already by update speed
    if (_frontend._speed_sensor_type.get() != _frontend.Speed_type::WINDSPEED_I2C) {
        update();
    }
}

void AP_WindVane_I2C::update_speed()
{
    update();
}

void AP_WindVane_I2C::update()
{
    WITH_SEMAPHORE(_sem);
    // read 6 bytes from the sensor
    uint8_t val[6];
    int ret = _dev->transfer(nullptr, 0, &val[0], sizeof(val));
    if (!ret) {
        return;
    }
    // for(size_t i = 0; i < 6; i++) {
    //     gcs().send_text(MAV_SEVERITY_INFO, "WNDVN DATA[%d]: %d", i, val[i]);
    // }
    

    // Check the CRC, see AP_Airspeed_SDP3X.cpp
    if (!_crc(&val[0], 2, val[2]) || !_crc(&val[3], 2, val[5])) {
        return;
    }

    // int16s are transmitted least-significant byte first.
    uint16_t dir = (((uint16_t)val[1]) << 8) | val[0];
    uint16_t speed = ((uint16_t) val[4]) << 8 | val[3];

    _wind_dir_deg = float(dir) *  360 / 65536.0;
    _speed_ms = float(speed);

    // user may not have I2C selected for both speed and direction
    if (_frontend._direction_type.get() == _frontend.WindVaneType::WINDVANE_I2C) {
        direction_update_frontend(wrap_PI(radians(_wind_dir_deg + _frontend._dir_analog_bearing_offset.get()) + AP::ahrs().yaw));
    }

    if (_frontend._speed_sensor_type.get() == _frontend.Speed_type::WINDSPEED_I2C) {
        speed_update_frontend(_speed_ms);
    }

}


/*
  check CRC for a set of bytes
 */
bool AP_WindVane_I2C::_crc(const uint8_t data[], unsigned size, uint8_t checksum)
{
    uint8_t crc_value = 0xff;

    // calculate 8-bit checksum with polynomial 0x31 (x^8 + x^5 + x^4 + 1)
    for (uint8_t i = 0; i < size; i++) {
        crc_value ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit) {
            if (crc_value & 0x80) {
                crc_value = (crc_value << 1) ^ 0x31;
            } else {
                crc_value = (crc_value << 1);
            }
        }
    }
    // verify checksum
    return (crc_value == checksum);
}