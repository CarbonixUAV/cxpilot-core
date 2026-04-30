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
   Abstract Telemetry library
*/

#include "AP_RCTelemetry.h"

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>
#include <math.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_Baro/AP_Baro.h>

#ifdef TELEM_DEBUG
# define debug(fmt, args...)	hal.console->printf("Telem: " fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

extern const AP_HAL::HAL& hal;

/*
  setup ready for passthrough telem
 */
bool AP_RCTelemetry::init(void)
{
#if HAL_GCS_ENABLED && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    // make telemetry available to GCS_MAVLINK (used to queue statustext messages from GCS_MAVLINK)
    // add firmware and frame info to message queue
    const char* _frame_string = gcs().frame_string();
    if (_frame_string == nullptr) {
        queue_message(MAV_SEVERITY_INFO, AP::fwversion().fw_string);
    } else {
        char firmware_buf[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
        snprintf(firmware_buf, sizeof(firmware_buf), "%s %s", AP::fwversion().fw_string, _frame_string);
        queue_message(MAV_SEVERITY_INFO, firmware_buf);
    }
#endif
    setup_wfq_scheduler();

    return true;
}

void AP_RCTelemetry::update_avg_packet_rate()
{
    uint32_t poll_now = AP_HAL::millis();

    _scheduler.avg_packet_counter++;

    if (poll_now - _scheduler.last_poll_timer > 1000) { //average in last 1000ms
        // initialize
        if (_scheduler.avg_packet_rate == 0) _scheduler.avg_packet_rate = _scheduler.avg_packet_counter;
        // moving average
        _scheduler.avg_packet_rate = (uint16_t)_scheduler.avg_packet_rate * 0.75f + _scheduler.avg_packet_counter * 0.25f;
        // reset
        _scheduler.last_poll_timer = poll_now;
        _scheduler.avg_packet_counter = 0;
        debug("avg packet rate %dHz, rates(Hz) %d %d %d %d %d %d %d %d", _scheduler.avg_packet_rate,
             _scheduler.packet_rate[0],
             _scheduler.packet_rate[1],
             _scheduler.packet_rate[2],
             _scheduler.packet_rate[3],
             _scheduler.packet_rate[4],
             _scheduler.packet_rate[5],
             _scheduler.packet_rate[6],
             _scheduler.packet_rate[7]);
    }
}

/*
 * WFQ scheduler
 * returns the actual packet type index (if any) sent by the scheduler
 */
uint8_t AP_RCTelemetry::run_wfq_scheduler(const bool use_shaper)
{
    // BITP layer 4 span 1: time the per-call prep work
    // (avg/max rate, sensor flags, ekf status, statustext semaphore)
    const uint32_t _bitp_prep_t0 = AP_HAL::micros();

    update_avg_packet_rate();
    update_max_packet_rate();

    uint32_t now = AP_HAL::millis();
    int8_t max_delay_idx = -1;

    float max_delay = 0;
    float delay = 0;
    bool packet_ready = false;

    // build message queue for sensor_status_flags
    check_sensor_status_flags();
    // build message queue for ekf_status
    check_ekf_status();

    // dynamic priorities
    bool queue_empty;
    {
        WITH_SEMAPHORE(_statustext.sem);
        queue_empty = !_statustext.available && _statustext.queue.is_empty();
    }

    adjust_packet_weight(queue_empty);

    const uint32_t _bitp_prep_us = AP_HAL::micros() - _bitp_prep_t0;
    if (_bitp_prep_us > 2000) {
        AP::logger().Write("BITP",
            "TimeUS,Layer,Span,Slot,Type,Bytes,Elapsed",
            "QBBbBHI",
            AP_HAL::micros64(),
            (uint8_t)4, (uint8_t)1, (int8_t)-1,
            (uint8_t)0, (uint16_t)0, _bitp_prep_us);
    }

    // BITP layer 4 span 2: time the slot-pick loop
    const uint32_t _bitp_pick_t0 = AP_HAL::micros();
    // search the packet with the longest delay after the scheduled time
    for (int i=0; i<_time_slots; i++) {
        // normalize packet delay relative to packet weight
        delay = (now - _scheduler.packet_timer[i])/static_cast<float>(_scheduler.packet_weight[i]);
        // use >= so with equal delays we choose the packet with lowest priority
        // this is ensured by the packets being sorted by desc frequency
        // apply the rate limiter
        if (delay >= max_delay && check_scheduler_entry_time_constraints(now, i, use_shaper)) {
            packet_ready = is_scheduler_entry_enabled(i) && is_packet_ready(i, queue_empty);

            if (packet_ready) {
                max_delay = delay;
                max_delay_idx = i;
            }
        }
    }
    const uint32_t _bitp_pick_us = AP_HAL::micros() - _bitp_pick_t0;
    if (_bitp_pick_us > 2000) {
        AP::logger().Write("BITP",
            "TimeUS,Layer,Span,Slot,Type,Bytes,Elapsed",
            "QBBbBHI",
            AP_HAL::micros64(),
            (uint8_t)4, (uint8_t)2, (int8_t)max_delay_idx,
            (uint8_t)0, (uint16_t)_time_slots, _bitp_pick_us);
    }

    if (max_delay_idx < 0) {  // nothing was ready
        return max_delay_idx;
    }

    now = AP_HAL::millis();
#ifdef TELEM_DEBUG
    _scheduler.packet_rate[max_delay_idx] = (_scheduler.packet_rate[max_delay_idx] + 1000 / (now - _scheduler.packet_timer[max_delay_idx])) / 2;
#endif
    _scheduler.packet_timer[max_delay_idx] = now;
    //debug("process_packet(%d): %f", max_delay_idx, max_delay);
    // BITP layer 4 span 3: time the actual frame builder call
    // (calc_battery / calc_attitude / calc_baro_vario / calc_vario / calc_gps / ...)
    // tagged with max_delay_idx so we can read off which scheduler slot was running
    const uint32_t _bitp_pp_t0 = AP_HAL::micros();
    process_packet(max_delay_idx);
    const uint32_t _bitp_pp_us = AP_HAL::micros() - _bitp_pp_t0;
    if (_bitp_pp_us > 2000) {
        AP::logger().Write("BITP",
            "TimeUS,Layer,Span,Slot,Type,Bytes,Elapsed",
            "QBBbBHI",
            AP_HAL::micros64(),
            (uint8_t)4, (uint8_t)3, (int8_t)max_delay_idx,
            (uint8_t)0, (uint16_t)0, _bitp_pp_us);
    }
    // let the caller know which packet type was sent
    return max_delay_idx;
}

/*
 * do not run the scheduler and process a specific entry
 */
bool AP_RCTelemetry::process_scheduler_entry(const uint8_t slot )
{
    if (slot >= TELEM_TIME_SLOT_MAX) {
        return false;
    }
    if (!is_scheduler_entry_enabled(slot)) {
        return false;
    }
    bool queue_empty;
    {
        WITH_SEMAPHORE(_statustext.sem);
        queue_empty = !_statustext.available && _statustext.queue.is_empty();
    }
    if (!is_packet_ready(slot, queue_empty)) {
        return false;
    }
    process_packet(slot);

    return true;
}

/*
 * add message to message cue for transmission through link
 */
void AP_RCTelemetry::queue_message(MAV_SEVERITY severity, const char *text)
{
    mavlink_statustext_t statustext{};

    statustext.severity = severity;
    strncpy_noterm(statustext.text, text, sizeof(statustext.text));

    // The force push will ensure comm links do not block other comm links forever if they fail.
    // If we push to a full buffer then we overwrite the oldest entry, effectively removing the
    // block but not until the buffer fills up.
    WITH_SEMAPHORE(_statustext.sem);
    _statustext.queue.push_force(statustext);
}

/*
 * add sensor_status_flags information to message cue, normally passed as sys_status mavlink messages to the GCS, for transmission through FrSky link
 */
void AP_RCTelemetry::check_sensor_status_flags(void)
{
    uint32_t now = AP_HAL::millis();

    const uint32_t _sensor_status_flags = sensor_status_flags();

    if ((now - check_sensor_status_timer) >= 5000) { // prevent repeating any system_status messages unless 5 seconds have passed
        // only one error is reported at a time (in order of preference). Same setup and displayed messages as Mission Planner.
        if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_GPS) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad GPS Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_3D_GYRO) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Gyro Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_3D_ACCEL) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Accel Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_3D_MAG) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Compass Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Baro Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_LASER_POSITION) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad LiDAR Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad OptFlow Health");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_TERRAIN) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad or No Terrain Data");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_GEOFENCE) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Geofence Breach");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_AHRS) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad AHRS");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_SENSOR_RC_RECEIVER) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "No RC Receiver");
            check_sensor_status_timer = now;
        } else if ((_sensor_status_flags & MAV_SYS_STATUS_LOGGING) > 0) {
            queue_message(MAV_SEVERITY_CRITICAL, "Bad Logging");
            check_sensor_status_timer = now;
        }
    }
}

/*
 * add innovation variance information to message cue, normally passed as ekf_status_report mavlink messages to the GCS, for transmission through FrSky link
 */
void AP_RCTelemetry::check_ekf_status(void)
{
    // get variances
    bool get_variance;
    float velVar, posVar, hgtVar, tasVar;
    Vector3f magVar;
    {
        AP_AHRS &_ahrs = AP::ahrs();
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        get_variance = _ahrs.get_variances(velVar, posVar, hgtVar, magVar, tasVar);
    }
    if (get_variance) {
        uint32_t now = AP_HAL::millis();
        if ((now - check_ekf_status_timer) >= 10000) { // prevent repeating any ekf_status message unless 10 seconds have passed
            // multiple errors can be reported at a time. Same setup as Mission Planner.
            if (velVar >= 0.8f) {
                queue_message(MAV_SEVERITY_CRITICAL, "Error velocity variance");
                check_ekf_status_timer = now;
            }
            if (posVar >= 0.8f) {
                queue_message(MAV_SEVERITY_CRITICAL, "Error pos horiz variance");
                check_ekf_status_timer = now;
            }
            if (hgtVar >= 0.8f) {
                queue_message(MAV_SEVERITY_CRITICAL, "Error pos vert variance");
                check_ekf_status_timer = now;
            }
            if (magVar.length() >= 0.8f) {
                queue_message(MAV_SEVERITY_CRITICAL, "Error compass variance");
                check_ekf_status_timer = now;
            }
            if (tasVar >= 0.8f) {
                queue_message(MAV_SEVERITY_CRITICAL, "Error terrain alt variance");
                check_ekf_status_timer = now;
            }
        }
    }
}

uint32_t AP_RCTelemetry::sensor_status_flags() const
{
    uint32_t present;
    uint32_t enabled;
    uint32_t health;
#if HAL_GCS_ENABLED
    gcs().get_sensor_status_flags(present, enabled, health);
#else
    present = 0;
    enabled = 0;
    health = 0;
#endif

    return ~health & enabled & present;
}

/*
 * get vertical speed from ahrs, if not available fall back to baro climbrate, units is m/s
 */
float AP_RCTelemetry::get_vspeed_ms(void)
{
    {
        // release semaphore as soon as possible
        AP_AHRS &_ahrs = AP::ahrs();
        Vector3f v;
        WITH_SEMAPHORE(_ahrs.get_semaphore());
        if (_ahrs.get_velocity_NED(v)) {
            return -v.z;
        }
    }
    auto &_baro = AP::baro();
    WITH_SEMAPHORE(_baro.get_semaphore());
    return _baro.get_climb_rate();
}

/*
 * prepare altitude between vehicle and home location data
 */
float AP_RCTelemetry::get_nav_alt_m(Location::AltFrame frame)
{
    Location loc;
    float current_height = 0;

    AP_AHRS &_ahrs = AP::ahrs();
    WITH_SEMAPHORE(_ahrs.get_semaphore());

    if (frame == Location::AltFrame::ABOVE_HOME) {
        _ahrs.get_relative_position_D_home(current_height);
        return -current_height;
    }

    if (_ahrs.get_location(loc)) {
        if (!loc.get_alt_m(frame, current_height)) {
            // ignore this error
        }
    }
    return current_height;
}
