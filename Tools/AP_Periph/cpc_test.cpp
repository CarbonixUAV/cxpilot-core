#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_CPC_STATUS

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#endif

/*
  Test-only toggling of CPC output pins from DroneCAN params (CPC_TOG_*).
  Each param is a toggle period in ms; 0 holds the pin at its hwdef default
  (HIGH = enabled), -1 holds it LOW (disabled). Only drives outputs - the
  resulting faults are reported by cpc_status.cpp as usual. All params are
  reset to 0 at boot.
 */

// drive one pin: toggle every period_ms, hold LOW at -1, or hold HIGH when the test is off
static void cpc_test_toggle(int16_t period_ms, ioline_t line, uint32_t &last_ms, uint32_t now_ms)
{
    if (period_ms <= 0) {
        palWriteLine(line, period_ms == -1 ? 0 : 1);
        last_ms = now_ms;
        return;
    }
    if (now_ms - last_ms >= uint32_t(period_ms)) {
        last_ms = now_ms;
        palToggleLine(line);
    }
}

void AP_Periph_FW::cpc_test_update()
{
    static bool boot_reset_done;
    if (!boot_reset_done) {
        boot_reset_done = true;
        g.cpc_tog_5v.set_and_save_ifchanged(0);
        g.cpc_tog_8v2.set_and_save_ifchanged(0);
        g.cpc_tog_stb.set_and_save_ifchanged(0);
        g.cpc_tog_pos.set_and_save_ifchanged(0);
    }

    const uint32_t now_ms = AP_HAL::millis();
    static uint32_t last_ms[4];

#ifdef HAL_GPIO_PIN_5V_PWR_EN
    cpc_test_toggle(g.cpc_tog_5v, HAL_GPIO_PIN_5V_PWR_EN, last_ms[0], now_ms);
#endif
#ifdef HAL_GPIO_PIN_8V2_PWR_EN
    cpc_test_toggle(g.cpc_tog_8v2, HAL_GPIO_PIN_8V2_PWR_EN, last_ms[1], now_ms);
#endif
#ifdef HAL_GPIO_PIN_LED_STROBE_EN
    cpc_test_toggle(g.cpc_tog_stb, HAL_GPIO_PIN_LED_STROBE_EN, last_ms[2], now_ms);
#endif
#ifdef HAL_GPIO_PIN_LED_POSITION_EN
    cpc_test_toggle(g.cpc_tog_pos, HAL_GPIO_PIN_LED_POSITION_EN, last_ms[3], now_ms);
#endif
}

#endif // HAL_PERIPH_ENABLE_CPC_STATUS
