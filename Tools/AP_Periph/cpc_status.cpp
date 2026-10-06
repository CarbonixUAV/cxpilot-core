#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_CPC_STATUS

#include <dronecan_msgs.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#endif

/*
  Carbonix CPC fault status, sent at 1Hz as a FlexDebug message for a Lua
  script on the flight controller to decode. Payload (version 1):
    [0] version
    [1] temp_alert:  bit0 8V2, bit1 5V, bit2 3V3, bit3 BRD
    [2] pgood_fault: bit0 5V, bit1 8V2, bit2 3V3
    [3] led_flt:     bit0 STROBE, bit1 POSITION
    [4] can_fault:   bit0 TCAN337 FAULT
  All bits are 1 = fault. Pins are sampled every loop and latched until the
  next send so faults shorter than the send interval are still reported.
 */

#define CPC_STATUS_FLEXDEBUG_ID 200
#define CPC_STATUS_VERSION 1
#define CPC_STATUS_SEND_INTERVAL_MS 1000

void AP_Periph_FW::cpc_status_update()
{
    const auto latch = [this](uint8_t idx, uint8_t bit, bool fault) {
        if (fault) {
            cpc_status_latch[idx] |= (1U << bit);
        }
    };

    // TMP1075 ALERT is active-low
#ifdef HAL_GPIO_PIN_8V2_TEMP_ALERT
    latch(0, 0, !palReadLine(HAL_GPIO_PIN_8V2_TEMP_ALERT));
#endif
#ifdef HAL_GPIO_PIN_5V_TEMP_ALERT
    latch(0, 1, !palReadLine(HAL_GPIO_PIN_5V_TEMP_ALERT));
#endif
#ifdef HAL_GPIO_PIN_3V3_TEMP_ALERT
    latch(0, 2, !palReadLine(HAL_GPIO_PIN_3V3_TEMP_ALERT));
#endif
#ifdef HAL_GPIO_PIN_BRD_TEMP_ALERT
    latch(0, 3, !palReadLine(HAL_GPIO_PIN_BRD_TEMP_ALERT));
#endif

    // PGOOD is HIGH when the rail is good
#ifdef HAL_GPIO_PIN_5V_PGOOD
    latch(1, 0, !palReadLine(HAL_GPIO_PIN_5V_PGOOD));
#endif
#ifdef HAL_GPIO_PIN_8V2_PGOOD
    latch(1, 1, !palReadLine(HAL_GPIO_PIN_8V2_PGOOD));
#endif
#ifdef HAL_GPIO_PIN_3V3_PGOOD
    latch(1, 2, !palReadLine(HAL_GPIO_PIN_3V3_PGOOD));
#endif

    // TPS27S100 FLT polarity not yet confirmed - raw pin level
#ifdef HAL_GPIO_PIN_LED_STROBE_FLT
    latch(2, 0, palReadLine(HAL_GPIO_PIN_LED_STROBE_FLT));
#endif
#ifdef HAL_GPIO_PIN_LED_POSITION_FLT
    latch(2, 1, palReadLine(HAL_GPIO_PIN_LED_POSITION_FLT));
#endif

    // TCAN337 FAULT is HIGH on fault
#ifdef HAL_GPIO_PIN_XCVR_BUS_FAULT
    latch(3, 0, palReadLine(HAL_GPIO_PIN_XCVR_BUS_FAULT));
#endif

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - cpc_status_last_send_ms < CPC_STATUS_SEND_INTERVAL_MS) {
        return;
    }
    cpc_status_last_send_ms = now_ms;

    dronecan_protocol_FlexDebug pkt {};
    pkt.id = CPC_STATUS_FLEXDEBUG_ID;
    pkt.u8.data[0] = CPC_STATUS_VERSION;
    memcpy(&pkt.u8.data[1], cpc_status_latch, sizeof(cpc_status_latch));
    pkt.u8.len = 1 + sizeof(cpc_status_latch);
    memset(cpc_status_latch, 0, sizeof(cpc_status_latch));

    uint8_t buffer[DRONECAN_PROTOCOL_FLEXDEBUG_MAX_SIZE];
    const uint16_t total_size = dronecan_protocol_FlexDebug_encode(&pkt, buffer, !canfdout());

    canard_broadcast(DRONECAN_PROTOCOL_FLEXDEBUG_SIGNATURE,
                     DRONECAN_PROTOCOL_FLEXDEBUG_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &buffer[0],
                     total_size);
}

#endif // HAL_PERIPH_ENABLE_CPC_STATUS
