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
#include <stdio.h> // debug
#include <AP_HAL/AP_HAL.h> //debug
#include <AP_BoardConfig/AP_BoardConfig_CAN.h> // debug

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

const AP_Param::GroupInfo OpticalFlow::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Optical flow sensor type
    // @Description: Optical flow sensor type
    // @Values: 0:None, 1:PX4Flow, 2:Pixart, 3:Bebop, 4:CXOF, 5:MAVLink, 6:UAVCAN
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("_TYPE", 0,  OpticalFlow,    _type,   (int8_t)OPTICAL_FLOW_TYPE_DEFAULT),

    // @Param: _FXSCALER
    // @DisplayName: X axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor X axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the X axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FXSCALER", 1,  OpticalFlow,    _flowScalerX,   0),

    // @Param: _FYSCALER
    // @DisplayName: Y axis optical flow scale factor correction
    // @Description: This sets the parts per thousand scale factor correction applied to the flow sensor Y axis optical rate. It can be used to correct for variations in effective focal length. Each positive increment of 1 increases the scale factor applied to the Y axis optical flow reading by 0.1%. Negative values reduce the scale factor.
    // @Range: -200 +200
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_FYSCALER", 2,  OpticalFlow,    _flowScalerY,   0),

    // @Param: _ORIENT_YAW
    // @DisplayName: Flow sensor yaw alignment
    // @Description: Specifies the number of centi-degrees that the flow sensor is yawed relative to the vehicle. A sensor with its X-axis pointing to the right of the vehicle X axis has a positive yaw angle.
    // @Range: -18000 +18000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ORIENT_YAW", 3,  OpticalFlow,    _yawAngle_cd,   0),

    // @Param: _POS_X
    // @DisplayName:  X position offset
    // @Description: X position of the optical flow sensor focal point in body frame. Positive X is forward of the origin.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced

    // @Param: _POS_Y
    // @DisplayName: Y position offset
    // @Description: Y position of the optical flow sensor focal point in body frame. Positive Y is to the right of the origin.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced

    // @Param: _POS_Z
    // @DisplayName: Z position offset
    // @Description: Z position of the optical flow sensor focal point in body frame. Positive Z is down from the origin.
    // @Units: m
    // @Range: -10 10
    // @User: Advanced
    AP_GROUPINFO("_POS", 4, OpticalFlow, _pos_offset, 0.0f),

    // @Param: _ADDR
    // @DisplayName: Address on the bus
    // @Description: This is used to select between multiple possible I2C addresses for some sensor types. For PX4Flow you can choose 0 to 7 for the 8 possible addresses on the I2C bus.
    // @Range: 0 127
    // @User: Advanced
    AP_GROUPINFO("_ADDR", 5,  OpticalFlow, _address,   0),

    // @Param: _ADDITIONAL
    // @DisplayName: Optical flow additional sensors
    // @Description: This indicates the number of additional optical flow sensors. Setting this to Disabled(0) will disable additional optical flow.
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard
    AP_GROUPINFO("_EXTRA", 6,  OpticalFlow,    _extra, 0),

    // @Param: _ORIENT_TILT
    // @DisplayName: Flow sensor tilt alignment
    // @Description: Specifies the number of centi-degrees that the additional flow sensors is tiled relative to the vertical axis of the vehicle.
    // @Range: 0 +9000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("_ORIENT_TILT", 7,  OpticalFlow,    _tiltAngle_cd,   0),

    AP_GROUPEND
};

// default constructor
OpticalFlow::OpticalFlow()
{
    _singleton = this;

    AP_Param::setup_object_defaults(this, var_info);
}

void OpticalFlow::init(uint32_t log_bit)
{
    hal.console->printf("Starting HereFlow\n");
     _log_bit = log_bit;
     num_instances = _extra+1;

    // return immediately if not enabled or backend already created
    if ((_type == (int8_t)OpticalFlowType::NONE) || (backend[0] != nullptr)) {
        return;
    }

    for (uint8_t i = 0; i<num_instances; i++) {

        switch ((OpticalFlowType)_type.get()) {
        case OpticalFlowType::NONE:
            break;
        case OpticalFlowType::PX4FLOW:
            backend[i] = AP_OpticalFlow_PX4Flow::detect(*this);
            break;
        case OpticalFlowType::PIXART:
            backend[i] = AP_OpticalFlow_Pixart::detect("pixartflow", *this);
            if (backend == nullptr) {
                backend[i] = AP_OpticalFlow_Pixart::detect("pixartPC15", *this);
            }
            break;
        case OpticalFlowType::BEBOP:
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BEBOP
            backend[i] = new AP_OpticalFlow_Onboard(*this);
#endif
            break;
        case OpticalFlowType::CXOF:
            backend[i] = AP_OpticalFlow_CXOF::detect(*this);
            break;
        case OpticalFlowType::MAVLINK:
            backend[i] = AP_OpticalFlow_MAV::detect(*this);
            break;
        case OpticalFlowType::UAVCAN:
#if HAL_WITH_UAVCAN
            break;
            //backend[i] = new AP_OpticalFlow_HereFlow(*this, i);
#endif
            break;
        case OpticalFlowType::SITL:
    #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            backend[i] = new AP_OpticalFlow_SITL(*this);
    #endif
            break;
        }

        if (backend[i] != nullptr) {
            backend[i]->init();
        }
    }
}

void OpticalFlow::update(void)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    for (uint8_t i = 0; i<num_instances; i++) {
        if (backend[i] != nullptr) {
            backend[i]->update();
        }
    }
    // only healthy if the data is less than 0.5s old
    _flags.healthy = (AP_HAL::millis() - _last_update_ms < 500*num_instances);
}

void OpticalFlow::handle_msg(const mavlink_message_t &msg)
{
    // exit immediately if not enabled
    if (!enabled()) {
        return;
    }

    for (uint8_t i = 0; i<num_instances; i++) {
        if (backend[i] != nullptr) {
            backend[i]->handle_msg(msg);
        }
    }
}

void OpticalFlow::update_state(const OpticalFlow_state &state)
{

    _state[0] = state;
    _last_update_ms = AP_HAL::millis();

    // write to log and send to EKF if new data has arrived
    AP::ahrs_navekf().writeOptFlowMeas(quality(),
                                       _state[0].flowRate,
                                       _state[0].bodyRate,
                                       _last_update_ms,
                                       get_pos_offset());
    Log_Write_Optflow(0);

}
void OpticalFlow::update_state2(const OpticalFlow_state &state, uint8_t instance)
{

    _state[instance] = state;
    states_new[instance] = true;

    all_true = true;

    for (uint8_t i = 0; i<num_instances; i++) {
        if (i == instance) { 
            continue; 
        } else if (!states_new[i]) {
            all_true = false;
            break;
        }
    }
    if (all_true) {
        for (uint8_t i = 0; i<num_instances; i++) {
            states_new[i] = false;
        }
        _last_update_ms = AP_HAL::millis();
    
        AP::ahrs_navekf().writeOptFlowMeas(quality(),
                                           _state[instance].flowRate,
                                           _state[instance].bodyRate,
                                           _last_update_ms,
                                           get_pos_offset());
        Log_Write_Optflow(0);
        Log_Write_Optflow(1);
    }
}

void OpticalFlow::Log_Write_Optflow(uint8_t instance)
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }
    if (_log_bit != (uint32_t)-1 &&
        !logger->should_log(_log_bit)) {
        return;
    }

    if (instance == 0) {
        struct log_Optflow pkt = {
            LOG_PACKET_HEADER_INIT(LOG_OPTFLOW_MSG),
            time_us         : AP_HAL::micros64(),
            surface_quality : _state[instance].surface_quality,
            flow_x          : _state[instance].flowRate.x,
            flow_y          : _state[instance].flowRate.y,
            body_x          : _state[instance].bodyRate.x,
            body_y          : _state[instance].bodyRate.y
        };
        logger->WriteBlock(&pkt, sizeof(pkt));
        return;
    }
    struct log_Optflow pkt = {
        LOG_PACKET_HEADER_INIT(LOG_OPTFLOW2_MSG),
        time_us         : AP_HAL::micros64(),
        surface_quality : _state[instance].surface_quality,
        flow_x          : _state[instance].flowRate.x,
        flow_y          : _state[instance].flowRate.y,
        body_x          : _state[instance].bodyRate.x,
        body_y          : _state[instance].bodyRate.y
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
