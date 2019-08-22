#include <AP_BoardConfig/AP_BoardConfig.h>
#include "OpticalFlow.h"
#include "AP_OpticalFlow_Onboard.h"
#include "AP_OpticalFlow_SITL.h"
#include "AP_OpticalFlow_Pixart.h"
#include "AP_OpticalFlow_PX4Flow.h"
#include "AP_OpticalFlow_CXOF.h"
#include "AP_OpticalFlow_MAV.h"
#include "AP_OpticalFlow_HereFlow.h"
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#ifndef OPTICAL_FLOW_TYPE_DEFAULT
 #if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_CHIBIOS_SKYVIPER_F412 || defined(HAL_HAVE_PIXARTFLOW_SPI)
  #define OPTICAL_FLOW_TYPE_DEFAULT OpticalFlowType::PIXART
 #elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
  #define OPTICAL_FLOW_TYPE_DEFAULT OpticalFlowType::BEBOP
 #else
  #define OPTICAL_FLOW_TYPE_DEFAULT OpticalFlowType::NONE
 #endif
#endif


// table of user settable parameters
const AP_Param::GroupInfo OpticalFlow::var_info[] = {

    // @Group: 1_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[0], "1_", 8, OpticalFlow, OpticalFlow_Params),

    // @Group: 1_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "1_",  17, OpticalFlow, backend_var_info[0]),

#if OPTICALFLOW_MAX_INSTANCES > 1
    // @Group: 2_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[1], "2_", 9, OpticalFlow, OpticalFlow_Params),

    // @Group: 2_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[1], "2_",  18, OpticalFlow, backend_var_info[1]),
#endif

#if OPTICALFLOW_MAX_INSTANCES > 2
    // @Group: 3_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[2], "3_", 10, OpticalFlow, OpticalFlow_Params),

    // @Group: 3_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[2], "3_",  19, OpticalFlow, backend_var_info[2]),

#if OPTICALFLOW_MAX_INSTANCES > 3
    // @Group: 4_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[3], "4_", 11, OpticalFlow, OpticalFlow_Params),

    // @Group: 4_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[0], "4_",  20, OpticalFlow, backend_var_info[3]),
#endif

#if OPTICALFLOW_MAX_INSTANCES > 4
    // @Group: 5_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[4], "5_", 12, OpticalFlow, OpticalFlow_Params),

    // @Group: 5_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[4], "5_",  21, OpticalFlow, backend_var_info[4]),
#endif

#if OPTICALFLOW_MAX_INSTANCES > 5
    // @Group: 6_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[5], "6_", 13, OpticalFlow, OpticalFlow_Params),

    // @Group: 6_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[5], "6_",  22, OpticalFlow, backend_var_info[5]),
#endif

#if OPTICALFLOW_MAX_INSTANCES > 6
    // @Group: 7_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[6], "7_", 14, OpticalFlow, OpticalFlow_Params),

    // @Group: 7_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[6], "7_",  23, OpticalFlow, backend_var_info[6]),
#endif

#if OPTICALFLOW_MAX_INSTANCES > 7
    // @Group: 8_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[7], "8_", 15, OpticalFlow, OpticalFlow_Params),

    // @Group: 8_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[7], "8_",  24, OpticalFlow, backend_var_info[7]),
#endif

#if OPTICALFLOW_MAX_INSTANCES > 8
    // @Group: 9_
    // @Path: AP_RangeFinder_Params.cpp
    AP_SUBGROUPINFO(params[8], "9_", 16, OpticalFlow, OpticalFlow_Params),

    // @Group: 9_
    // @Path: AP_RangeFinder_Wasp.cpp
    AP_SUBGROUPVARPTR(drivers[8], "9_",  25, OpticalFlow, backend_var_info[8]),
#endif

    AP_GROUPEND
};

const AP_Param::GroupInfo *OpticalFlow::backend_var_info[9];


// const AP_Param::GroupInfo OpticalFlow::var_info[] = {
//     // @Param: _TYPE
//     // @DisplayName: Optical flow sensor type
//     // @Description: Optical flow sensor type
//     // @Values: 0:None, 1:PX4Flow, 2:Pixart, 3:Bebop, 4:CXOF, 5:MAVLink, 6:UAVCAN
//     // @User: Standard
//     // @RebootRequired: True
//     AP_GROUPINFO("_TYPE", 0,  OpticalFlow,    _type,   (int8_t)OPTICAL_FLOW_TYPE_DEFAULT),

//     // @Param: _FXSCALER
//     // @DisplayName: X axis optical flow scale factor correction
//     // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0.1%. Negative values reduce the scale factor.
//     // @Range: -200 +200
//     // @Increment: 1
//     // @User: Standard
//     AP_GROUPINFO("_FXSCALER", 1,  OpticalFlow,    _flowScalerX,   0),

//     // @Param: _FYSCALER
//     // @DisplayName: Y axis optical flow scale factor correction
//     // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0.1%. Negative values reduce the scale factor.
//     // @Range: -200 +200
//     // @Increment: 1
//     // @User: Standard
//     AP_GROUPINFO("_FYSCALER", 2,  OpticalFlow,    _flowScalerY,   0),

//     // @Param: _ORIENT_YAW
//     // @DisplayName: Flow sensor yaw alignment
//     // @Description: Specifies the number of centi-degrees that the flow sensor is yawed relative to the vehicle. A sensor with its X-axis pointing to the right of the vehicle X axis has a positive yaw angle.
//     // @Range: -18000 +18000
//     // @Increment: 1
//     // @User: Standard
//     AP_GROUPINFO("_ORIENT_YAW", 3,  OpticalFlow,    _yawAngle_cd,   0),

//     // @Param: _POS_X
//     // @DisplayName:  X position offset
//     // @Description: X position of the optical flow sensor focal point in body frame. Positive X is forward of the origin.
//     // @Units: m
//     // @Range: -10 10
//     // @User: Advanced

//     // @Param: _POS_Y
//     // @DisplayName: Y position offset
//     // @Description: Y position of the optical flow sensor focal point in body frame. Positive Y is to the right of the origin.
//     // @Units: m
//     // @Range: -10 10
//     // @User: Advanced

//     // @Param: _POS_Z
//     // @DisplayName: Z position offset
//     // @Description: Z position of the optical flow sensor focal point in body frame. Positive Z is down from the origin.
//     // @Units: m
//     // @Range: -10 10
//     // @User: Advanced
//     AP_GROUPINFO("_POS", 4, OpticalFlow, _pos_offset, 0.0f),

//     // @Param: _ADDR
//     // @DisplayName: Address on the bus
//     // @Description: This is used to select between multiple possible I2C addresses for some sensor types. For PX4Flow you can choose 0 to 7 for the 8 possible addresses on the I2C bus.
//     // @Range: 0 127
//     // @User: Advanced
//     AP_GROUPINFO("_ADDR", 5,  OpticalFlow, _address,   0),

//     // the parameter description below is for GCSs (like MP) that use master for the parameter descriptions.  This should be removed when Copter-3.7 is released
//     // @Param: _ENABLE
//     // @DisplayName: Optical flow enable/disable
//     // @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
//     // @Values: 0:Disabled, 1:Enabled
//     // @User: Standard

//     AP_GROUPEND
// };

// default constructor
OpticalFlow::OpticalFlow()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

void OpticalFlow::init(uint32_t log_bit)
{
    if (num_instances != 0) {
        // init called a 2nd time?
        return;
    }
    
     _log_bit = log_bit;

    // return immediately if not enabled or backend already created
    if ((_type == (int8_t)params[0].type == NONE) || (backend != nullptr)) {
        return;
    }

        for (uint8_t i=0, serial_instance = 0; i<RANGEFINDER_MAX_INSTANCES; i++) {
        // serial_instance will be increased inside detect_instance
        // if a serial driver is loaded for this instance
        detect_instance(i, serial_instance);
        if (drivers[i] != nullptr) {
            // we loaded a driver for this instance, so it must be
            // present (although it may not be healthy)
            num_instances = i+1;
        }

        // initialise status
        state[i].status = RangeFinder_NotConnected;
        state[i].range_valid_count = 0;
    }

    switch ((OpticalFlowType)_type.get()) {
    case OpticalFlowType::NONE:
        break;
    case OpticalFlowType::PX4FLOW:
        backend = AP_OpticalFlow_PX4Flow::detect(*this);
        break;
    case OpticalFlowType::PIXART:
        backend = AP_OpticalFlow_Pixart::detect("pixartflow", *this);
        if (backend == nullptr) {
            backend = AP_OpticalFlow_Pixart::detect("pixartPC15", *this);
        }
        break;
    case OpticalFlowType::BEBOP:
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
        backend = new AP_OpticalFlow_Onboard(*this);
#endif
        break;
    case OpticalFlowType::CXOF:
        backend = AP_OpticalFlow_CXOF::detect(*this);
        break;
    case OpticalFlowType::MAVLINK:
        backend = AP_OpticalFlow_MAV::detect(*this);
        break;
    case OpticalFlowType::UAVCAN:
#if HAL_WITH_UAVCAN
        backend = new AP_OpticalFlow_HereFlow(*this);
#endif
        break;
    case OpticalFlowType::SITL:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        backend = new AP_OpticalFlow_SITL(*this);
#endif
        break;
    }

    if (backend != nullptr) {
        backend->init();
    }
}

void OpticalFlow::update(void)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }
    if (backend != nullptr) {
        backend->update();
    }

    // only healthy if the data is less than 0.5s old
    _flags.healthy = (AP_HAL::millis() - _last_update_ms < 500);
}

void OpticalFlow::handle_msg(const mavlink_message_t &msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    if (backend != nullptr) {
        backend->handle_msg(msg);
    }
}

void OpticalFlow::update_state(const OpticalFlow_state &state)
{
    _state = state;
    _last_update_ms = AP_HAL::millis();

    // write to log and send to EKF if new data has arrived
    AP::ahrs_navekf().writeOptFlowMeas(quality(),
                                       _state.flowRate,
                                       _state.bodyRate,
                                       _last_update_ms,
                                       get_pos_offset());
    Log_Write_Optflow();
}

void OpticalFlow::Log_Write_Optflow()
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }
    if (_log_bit != (uint32_t)-1 &&
        !logger->should_log(_log_bit)) {
        return;
    }

    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
        time_us         : AP_HAL::micros64(),
        surface_quality : _state.surface_quality,
        flow_x          : _state.flowRate.x,
        flow_y          : _state.flowRate.y,
        body_x          : _state.bodyRate.x,
        body_y          : _state.bodyRate.y
    };
    logger->WriteBlock(&pkt, sizeof(pkt));
}



// singleton instance
OpticalFlow *OpticalFlow::_singleton;

namespace AP {

OpticalFlow *opticalflow()
{
    return OpticalFlow::get_singleton();
}

}
