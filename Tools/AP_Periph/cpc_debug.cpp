#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_CPC_DEBUG_PRINT

extern const AP_HAL::HAL &hal;

/*
  Carbonix CPC bring-up debug, called at 1Hz. Prints all battery monitor
  voltages and temperature sensor readings to the DroneCAN log console,
  labelled by HAL_PERIPH_CPC_BATT_LABELS / HAL_PERIPH_CPC_TEMP_LABELS.
 */
void AP_Periph_FW::cpc_debug_print()
{
#ifdef HAL_PERIPH_ENABLE_BATTERY
    static const char *const batt_labels[] = HAL_PERIPH_CPC_BATT_LABELS;
    char vbuf[80] {};
    uint8_t vlen = 0;
    const uint8_t num_batt = MIN(battery_lib.num_instances(), ARRAY_SIZE(batt_labels));
    for (uint8_t i = 0; i < num_batt && vlen < sizeof(vbuf); i++) {
        vlen += hal.util->snprintf(&vbuf[vlen], sizeof(vbuf) - vlen, " %s:%.2f", batt_labels[i], battery_lib.voltage(i));
    }
    can_printf("V%s", vbuf);
#endif

#if AP_TEMPERATURE_SENSOR_ENABLED
    static const char *const temp_labels[] = HAL_PERIPH_CPC_TEMP_LABELS;
    char tbuf[80] {};
    uint8_t tlen = 0;
    const uint8_t num_temp = MIN(temperature_sensor.num_instances(), ARRAY_SIZE(temp_labels));
    for (uint8_t i = 0; i < num_temp && tlen < sizeof(tbuf); i++) {
        float temp;
        if (temperature_sensor.get_temperature(temp, i)) {
            tlen += hal.util->snprintf(&tbuf[tlen], sizeof(tbuf) - tlen, " %s:%.1f", temp_labels[i], temp);
        } else {
            tlen += hal.util->snprintf(&tbuf[tlen], sizeof(tbuf) - tlen, " %s:--", temp_labels[i]);
        }
    }
    can_printf("T%s", tbuf);
#endif
}

#endif // HAL_PERIPH_ENABLE_CPC_DEBUG_PRINT
