#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_PMU_TELEM

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

/*
  Plettenberg S-GCU starter-generator controller RS232 telemetry, 115200 8N1,
  receive only.
  Line format:
    T=3.649V,a=0.000V,PWM= 787,U= 34.9V,I=   3.7A,RPM=  1482,con= 28°C,mot= 26°C
  First key is T (analog), t (brake active) or S (serial control); the aux key
  is a, or A when the brake is active. Other lines (power-up banner) are
  ignored. Never write to this UART: the S-GCU accepts single-byte throttle
  commands.

  Decoded values are sent as a FlexDebug message for a Lua script on the
  flight controller: on each good line (max 10Hz), or at 1Hz with the stale
  bit set when no good line arrived for 1s. Payload (version 1, little-endian):
    [0]  u8  version
    [1]  u8  status: bit0 stale, bit1 parse error since last send, bit2 brake,
             bit3-4 mode (0 T, 1 t, 2 S)
    [2]  u16 throttle input mV
    [4]  u16 aux input mV
    [6]  u16 PWM duty cycle, permille (0-1000 = 0-100%)
    [8]  u16 input voltage 0.1V
    [10] i16 phase current 0.1A
    [12] u32 RPM
    [16] i8  controller temperature degC
    [17] i8  motor temperature degC
    [18] u16 parse error count
 */

#define PMU_TELEM_BAUD 115200
#define PMU_TELEM_STALE_MS 1000
#define PMU_TELEM_MIN_SEND_MS 100
#define PMU_TELEM_STALE_SEND_MS 1000
#define PMU_TELEM_FLEXDEBUG_ID 201
#define PMU_TELEM_VERSION 1
#define PMU_TELEM_RPM_MAX 1e7f

struct PACKED pmu_telem_payload {
    uint8_t version;
    uint8_t status;
    uint16_t throttle_mv;
    uint16_t aux_mv;
    uint16_t pwm;
    uint16_t voltage_dv;
    int16_t current_da;
    uint32_t rpm;
    int8_t temp_con;
    int8_t temp_mot;
    uint16_t err_count;
};
static_assert(sizeof(pmu_telem_payload) == 20, "PMU payload layout changed");

void AP_Periph_FW::pmu_telem_init()
{
    if (g.pmu_port < 0) {
        return;
    }
    pmu.uart = hal.serial(g.pmu_port);
    if (pmu.uart != nullptr) {
        pmu.uart->begin(PMU_TELEM_BAUD);
    }
}

// parse pmu.line in place, true if all fields were decoded
bool AP_Periph_FW::pmu_telem_parse_line()
{
    static const char *const keys[] = { "T", "a", "PWM", "U", "I", "RPM", "con", "mot" };
    float val[ARRAY_SIZE(keys)];
    uint16_t found = 0;
    bool brake = false;
    const char mode = pmu.line[0];

    char *saveptr = nullptr;
    for (char *field = strtok_r(pmu.line, ",", &saveptr); field != nullptr; field = strtok_r(nullptr, ",", &saveptr)) {
        char *eq = strchr(field, '=');
        if (eq == nullptr) {
            return false;
        }
        *eq = 0;
        const char *key = field;
        if (strcmp(key, "t") == 0 || strcmp(key, "S") == 0) {
            key = "T";
        } else if (strcmp(key, "A") == 0) {
            key = "a";
            brake = true;
        }
        char *end;
        const float v = strtof(eq + 1, &end);
        if (end == eq + 1 || !isfinite(v)) {
            return false;
        }
        for (uint8_t i = 0; i < ARRAY_SIZE(keys); i++) {
            if (strcmp(key, keys[i]) == 0) {
                val[i] = v;
                found |= (1U << i);
                break;
            }
        }
    }
    if (found != (1U << ARRAY_SIZE(keys)) - 1) {
        return false;
    }

    pmu.mode = mode;
    pmu.brake = brake;
    pmu.throttle_v = val[0];
    pmu.aux_v = val[1];
    // clamp before integer conversion, out-of-range float to int is undefined
    pmu.pwm = uint16_t(constrain_float(val[2], 0, UINT16_MAX));
    pmu.voltage = val[3];
    pmu.current = val[4];
    pmu.rpm = uint32_t(constrain_float(val[5], 0, PMU_TELEM_RPM_MAX));
    pmu.temp_con = int16_t(constrain_float(val[6], INT16_MIN, INT16_MAX));
    pmu.temp_mot = int16_t(constrain_float(val[7], INT16_MIN, INT16_MAX));
    return true;
}

void AP_Periph_FW::pmu_telem_update()
{
    if (pmu.uart == nullptr) {
        return;
    }

    const uint32_t now_ms = AP_HAL::millis();
    uint32_t n = pmu.uart->available();
    while (n--) {
        uint8_t b;
        if (!pmu.uart->read(b)) {
            break;
        }
        if (b == '\r' || b == '\n') {
            if (pmu.len == 0) {
                continue;
            }
            pmu.line[pmu.len] = 0;
            const bool telem_line = pmu.len >= 2 && pmu.line[1] == '=' &&
                                    (pmu.line[0] == 'T' || pmu.line[0] == 't' || pmu.line[0] == 'S');
            if (telem_line) {
                if (pmu_telem_parse_line()) {
                    pmu.good_count++;
                    pmu.last_good_ms = now_ms;
                    pmu.new_line = true;
                } else {
                    pmu.err_count++;
                    pmu.err_since_send = true;
                }
            }
            pmu.len = 0;
            continue;
        }
        if (pmu.len >= sizeof(pmu.line) - 1) {
            // no line end within buffer, drop it
            pmu.len = 0;
            pmu.err_count++;
            pmu.err_since_send = true;
        }
        pmu.line[pmu.len++] = b;
    }

    const bool stale = pmu.good_count == 0 || now_ms - pmu.last_good_ms > PMU_TELEM_STALE_MS;
    const uint32_t since_send_ms = now_ms - pmu.last_send_ms;
    if (pmu.new_line && !stale && since_send_ms >= PMU_TELEM_MIN_SEND_MS) {
        pmu_telem_send(false);
    } else if (stale && since_send_ms >= PMU_TELEM_STALE_SEND_MS) {
        pmu_telem_send(true);
    }
}

void AP_Periph_FW::pmu_telem_send(bool stale)
{
    uint8_t mode = 0;
    if (pmu.mode == 't') {
        mode = 1;
    } else if (pmu.mode == 'S') {
        mode = 2;
    }

    pmu_telem_payload p {};
    p.version = PMU_TELEM_VERSION;
    p.status = (stale ? 1U : 0U) | (pmu.err_since_send ? 2U : 0U) | (pmu.brake ? 4U : 0U) | (mode << 3);
    p.throttle_mv = uint16_t(lrintf(constrain_float(pmu.throttle_v * 1000, 0, UINT16_MAX)));
    p.aux_mv = uint16_t(lrintf(constrain_float(pmu.aux_v * 1000, 0, UINT16_MAX)));
    p.pwm = pmu.pwm;
    p.voltage_dv = uint16_t(lrintf(constrain_float(pmu.voltage * 10, 0, UINT16_MAX)));
    p.current_da = int16_t(lrintf(constrain_float(pmu.current * 10, INT16_MIN, INT16_MAX)));
    p.rpm = pmu.rpm;
    p.temp_con = constrain_int16(pmu.temp_con, INT8_MIN, INT8_MAX);
    p.temp_mot = constrain_int16(pmu.temp_mot, INT8_MIN, INT8_MAX);
    p.err_count = MIN(pmu.err_count, UINT16_MAX);

    dronecan_protocol_FlexDebug pkt {};
    pkt.id = PMU_TELEM_FLEXDEBUG_ID;
    memcpy(pkt.u8.data, &p, sizeof(p));
    pkt.u8.len = sizeof(p);

    uint8_t buffer[DRONECAN_PROTOCOL_FLEXDEBUG_MAX_SIZE];
    const uint16_t total_size = dronecan_protocol_FlexDebug_encode(&pkt, buffer, !canfdout());

    canard_broadcast(DRONECAN_PROTOCOL_FLEXDEBUG_SIGNATURE,
                     DRONECAN_PROTOCOL_FLEXDEBUG_ID,
                     CANARD_TRANSFER_PRIORITY_LOW,
                     &buffer[0],
                     total_size);

    pmu.last_send_ms = AP_HAL::millis();
    pmu.new_line = false;
    pmu.err_since_send = false;
}

#endif // HAL_PERIPH_ENABLE_PMU_TELEM
