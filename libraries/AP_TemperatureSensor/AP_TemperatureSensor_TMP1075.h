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

#pragma once
#include "AP_TemperatureSensor_Backend.h"

#ifndef AP_TEMPERATURE_SENSOR_TMP1075_ENABLED
    #define AP_TEMPERATURE_SENSOR_TMP1075_ENABLED AP_TEMPERATURE_SENSOR_ENABLED
#endif

#if AP_TEMPERATURE_SENSOR_TMP1075_ENABLED

#define TMP1075_I2CDEFAULTADDR 0x48

class AP_TemperatureSensor_TMP1075 : public AP_TemperatureSensor_Backend {
    using AP_TemperatureSensor_Backend::AP_TemperatureSensor_Backend;
public:

    void init(void) override;

    void update(void) override {};

private:

    void _timer(void);

    bool read_temperature(float &temperature) const;
};

#endif // AP_TEMPERATURE_SENSOR_TMP1075_ENABLED
