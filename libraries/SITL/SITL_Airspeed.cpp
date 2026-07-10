#include "SITL.h"

#if AP_SIM_ENABLED

namespace SITL {
// user settable parameters for airspeed sensors
const AP_Param::GroupInfo SIM::AirspeedParm::var_info[] = {
        // user settable parameters for the 1st airspeed sensor
    AP_GROUPINFO("RND",     1, AirspeedParm,  noise, 2.0),
    AP_GROUPINFO("OFS",     2, AirspeedParm,  offset, 2013),
    // @Param: FAIL
    // @DisplayName: Airspeed sensor failure
    // @Description: Simulates Airspeed sensor 1 failure
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("FAIL",    3, AirspeedParm,  fail, 0),
    // @Param: FAILP
    // @DisplayName: Airspeed sensor failure pressure
    // @Description: Simulated airspeed sensor failure pressure
    // @Units: Pa
    // @User: Advanced
    AP_GROUPINFO("FAILP",   4, AirspeedParm,  fail_pressure, 0),
    // @Param: PITOT
    // @DisplayName: Airspeed pitot tube failure pressure
    // @Description: Simulated airspeed sensor pitot tube failure pressure
    // @Units: Pa
    // @User: Advanced
    AP_GROUPINFO("PITOT",   5, AirspeedParm,  fail_pitot_pressure, 0),
    // @Param: SIGN
    // @DisplayName: Airspeed signflip
    // @Description: Simulated airspeed sensor with reversed pitot/static connections
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO("SIGN",    6, AirspeedParm,  signflip, 0),
    // @Param: RATIO
    // @DisplayName: Airspeed ratios
    // @Description: Simulated airspeed sensor ratio
    // @User: Advanced
    AP_GROUPINFO("RATIO",   7, AirspeedParm,  ratio, 1.99),
    // @Param: POS_Y
    // @DisplayName: Airspeed sensor Y position offset
    // @Description: Simulated lateral (body Y) mounting offset of the pitot, +Y is to the right of the vehicle. The simulated reading gains the tangential velocity a pitot at this offset would see during yaw, so it can be cancelled by ARSPDn_POS_Y.
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("POS_Y",   8, AirspeedParm,  pos_y, 0),
    AP_GROUPEND
};
}

#endif  // AP_SIM_ENABLED