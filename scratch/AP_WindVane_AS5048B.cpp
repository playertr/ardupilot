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
#include "AP_WindVane_AS5048B.h"
#include <AP_HAL/utility/OwnPtr.h>
#include <GCS_MAVLink/GCS.h>

#define I2C_BUS 0

#define RE_ADDRESS          0x40
#define RE_ZEROMSB_REG      0x16  
// Zero, most significant byte
#define RE_ZEROLSB_REG      0x17  
// Zero, least significant byte
#define RE_MAGNMSB_REG      0xFC  
// Magnitude, most significant byte
#define RE_MAGNLSB_REG      0xFD  
// Magnitude, least significant byte
#define RE_ANGLEMSB_REG     0xFE  
// Angle, most significant byte
#define RE_ANGLELSB_REG     0xFF  
// Angle, least significant byte

extern const AP_HAL::HAL& hal;
 
// init - performs any required initialization for this instance
void AP_WindVane_AS5048B::init()
{   

    _dev = hal.i2c_mgr->get_device(I2C_BUS, RE_ADDRESS);
    if (!_dev) {
        gcs().send_text(MAV_SEVERITY_INFO, "No AP_WindVane_AS5048B found.");
        return;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "AP_WindVane_AS5048B: Found on bus %u address 0x%02x",
                        I2C_BUS, RE_ADDRESS);

    if(!_dev->get_semaphore()->take(10)){
        return;
    }
    calibrate();
    _dev->get_semaphore()->give();

}

void AP_WindVane_AS5048B::calibrate()
{
    _dev->set_retries(10);
    // set initial position to zero by setting the zero register to zero,
    // reading the measured angle, and setting the register to this angle.
    int ret = _dev->write_register(RE_ZEROMSB_REG, 0x00);
    if (!ret) {
        return;
    }
    ret = _dev->write_register(RE_ZEROLSB_REG, 0x00);
    if (!ret) {
        return;
    }
    uint8_t msb = 0;
    ret = _dev->read_registers(RE_ANGLEMSB_REG, &msb, 1);
    if (!ret) {
        return;
    }
    uint8_t lsb = 0;
    ret = _dev->read_registers(RE_ANGLELSB_REG, &lsb, 1);
    if (!ret) {
        return;
    }
    ret = _dev->write_register(RE_ZEROMSB_REG, msb);
    if (!ret) {
        return;
    }
    ret = _dev->write_register(RE_ZEROLSB_REG, lsb);
    if (!ret) {
        return;
    }
    _dev->set_retries(2);
}

void AP_WindVane_AS5048B::update_direction()
{
    if(!_dev->get_semaphore()->take(2)){
        return;
    }

    uint8_t msb = 0;
    int ret = _dev->read_registers(RE_ANGLEMSB_REG, &msb, 1);
    if (!ret) {
        return;
    }
    uint8_t lsb = 0;
    ret = _dev->read_registers(RE_ANGLELSB_REG, &lsb, 1);
    if (!ret) {
        return;
    }

    // bit shifting, see https://ams.com/documents/20143/36005/AS5048_DS000298_4-00.pdf p. 25
    uint16_t angle = ((uint16_t) msb << 6) | (lsb & 0x3f);
    _wind_dir_deg = (float) angle * 360.0 / 16384.0;

    // user may not have AS5048B selected for direction, I think.
    if (_frontend._direction_type.get() == _frontend.WindVaneType::WINDVANE_AS5048B) {
        _frontend._direction_apparent_raw = wrap_PI(radians(_wind_dir_deg));
    }

    _dev->get_semaphore()->give();
}
