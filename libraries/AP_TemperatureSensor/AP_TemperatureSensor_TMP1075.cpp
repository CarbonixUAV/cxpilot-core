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
/*
    Implements I2C driver for Texas Instruments TMP1075 digital temperature sensor
*/

#include "AP_TemperatureSensor_TMP1075.h"

#if AP_TEMPERATURE_SENSOR_TMP1075_ENABLED

#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

extern const AP_HAL::HAL &hal;

// register map per TI TMP1075 datasheet (SBOS854F) section 7.5:
//   0x00 TEMP  - last conversion result: 12-bit, left-justified in a
//                16-bit two's-complement word, 0.0625 degC/LSB
//   0x0F DIEID - read-only, MSB always 0x75, used here as an init-time
//                sanity check that the device is actually a TMP1075
// Relies on the TMP1075's power-on default of continuous conversion mode
// (~27.5ms max conversion time) - no config register writes needed.
static const uint8_t TMP1075_REG_TEMP = 0x00;
static const uint8_t TMP1075_REG_DIEID = 0x0F;
static const uint8_t TMP1075_DIEID_MSB = 0x75;

#define AP_TemperatureSensor_TMP1075_UPDATE_INTERVAL_MS 100
#define AP_TemperatureSensor_TMP1075_SCALE_FACTOR        (0.0625f)

void AP_TemperatureSensor_TMP1075::init()
{
    _params.bus_address.set_default(TMP1075_I2CDEFAULTADDR);

    _dev = std::move(hal.i2c_mgr->get_device(_params.bus, _params.bus_address));
    if (!_dev) {
        // device not found
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    // sanity check: confirm the device is actually a TMP1075 before
    // trusting it, rather than just checking the transfer succeeded
    uint8_t dieid[2];
    if (!_dev->read_registers(TMP1075_REG_DIEID, dieid, sizeof(dieid)) ||
        dieid[0] != TMP1075_DIEID_MSB) {
        return;
    }

    // lower retries for run
    _dev->set_retries(3);

    _dev->register_periodic_callback(AP_TemperatureSensor_TMP1075_UPDATE_INTERVAL_MS * AP_USEC_PER_MSEC,
                                      FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_TMP1075::_timer, void));
}

void AP_TemperatureSensor_TMP1075::_timer()
{
    float temperature;
    if (read_temperature(temperature)) {
        set_temperature(temperature);
    }
}

bool AP_TemperatureSensor_TMP1075::read_temperature(float &temperature) const
{
    uint8_t data[2];
    if (!_dev->read_registers(TMP1075_REG_TEMP, data, sizeof(data))) {
        return false;
    }

    const int16_t raw = int16_t(UINT16_VALUE(data[0], data[1])) >> 4;
    temperature = raw * AP_TemperatureSensor_TMP1075_SCALE_FACTOR;

    return true;
}

#endif // AP_TEMPERATURE_SENSOR_TMP1075_ENABLED
