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
#include "AP_WindVane_AS5600.h"
#include <AP_HAL/utility/OwnPtr.h>
#include <GCS_MAVLink/GCS.h>

#define I2C_BUS 1

#define RE_ADDRESS          0x36
#define RE_RAW_ANGLE        0x0C
#define AS5600_CONF         0x07

//  CONFIGURATION BIT MASKS - byte level
#define AS5600_CONF_POWER_MODE      0x03;

extern const AP_HAL::HAL& hal;
 
// init - performs any required initialization for this instance
void AP_WindVane_AS5600::init()
{   

    // Defaults to absurdly high latency low power mode
    _dev = hal.i2c_mgr->get_device(I2C_BUS, RE_ADDRESS); //, 400000U, false, 105);
    if (!_dev) {
        gcs().send_text(MAV_SEVERITY_INFO, "No AP_WindVane_AS5600 found.");
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "AP_WindVane_AS5600: Found on bus %u address 0x%02x",
                        I2C_BUS, RE_ADDRESS);

    _dev->set_retries(2);

    setPowerMode(0x00);

    calibrate();


}

// https://github.com/RobTillaart/AS5600/blob/master/AS5600.cpp
bool AP_WindVane_AS5600::setPowerMode(uint8_t powerMode)
{
  if (powerMode > 3) return false;

  _dev->set_retries(10);

  WITH_SEMAPHORE(_dev->get_semaphore());

  uint8_t value;
  if (not _dev->read_registers((AS5600_CONF + 1), &value, 1))
  {
    gcs().send_text(MAV_SEVERITY_INFO, "AP_WindVane_AS5600: Could not get CONF value to set power.");
    _dev->set_retries(2);
    return false;
  }
  gcs().send_text(MAV_SEVERITY_INFO, "AP_WindVane_AS5600: Got CONF value %02x",
                        value);

  value &= ~AS5600_CONF_POWER_MODE;
  value |= powerMode;

  bool written = _dev->write_register((AS5600_CONF + 1), value, true);
  if (written)
  {
    gcs().send_text(MAV_SEVERITY_INFO, "AP_WindVane_AS5600: Set CONF value %02x",
                        value);
  }
  else
  {
    gcs().send_text(MAV_SEVERITY_INFO, "AP_WindVane_AS5600: Could not set CONF value %02x",
                        value);
    _dev->set_retries(2);
    return false;
  }
  
  _dev->set_retries(2);

  return true;
}

uint16_t AP_WindVane_AS5600::readRawAngle()
{

    WITH_SEMAPHORE(_dev->get_semaphore());

    // Read two-byte angle (12-bit precision) and apply offset
    uint16_t value;
    if (!_dev->read_registers(RE_RAW_ANGLE, (uint8_t *) &value, 2)) {
        gcs().send_text(MAV_SEVERITY_INFO,"AP_WindVane_AS5600: Reading the angle register failed.");
        return 0x0FFF;
    }

    // gcs().send_text(MAV_SEVERITY_INFO,"AP_WindVane_AS5600: Got angle bytes %x.", value);

    // Swap the bytes. Extract the first byte and shift it right. Extract the second byte 
    // and shift it left. Bitwise OR the them together and you're set!
    value = ((value & 0xff) << 8) | ((value & 0xff00) >> 8);

    // Due to twelve-bit precision, we make sure we have a twelve-bit angle.
    value &=  0x0FFF;

    // gcs().send_text(MAV_SEVERITY_INFO,"AP_WindVane_AS5600: Interpreting angle %u.", value);

    return value;
}

void AP_WindVane_AS5600::calibrate()
{
    _raw_angle_offset = readRawAngle();
}

void AP_WindVane_AS5600::update_direction()
{

    _wind_dir_rad = (float) (readRawAngle() + _raw_angle_offset) * 2.0 * M_PI / 4096.0;

    _frontend._direction_apparent_raw = wrap_PI(_wind_dir_rad);

    // gcs().send_text(MAV_SEVERITY_INFO, "Setting wind direction %f", _frontend._direction_apparent_raw);

}
