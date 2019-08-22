#include "OpticalFlow_Params.h"
#include "OpticalFlow.h"

// table of user settable parameters
const AP_Param::GroupInfo OpticalFlow::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: Optical flow sensor type
    // @Description: Optical flow sensor type
    // @Values: 0:None, 1:PX4Flow, 2:Pixart, 3:Bebop, 4:CXOF, 5:MAVLink, 6:UAVCAN
    // @User: Standard
    // @RebootRequired: True
    // AP_GROUPINFO("_TYPE", 0,  OpticalFlow,    _type,   (int8_t)OPTICAL_FLOW_TYPE_DEFAULT),
    AP_GROUPINFO("_TYPE", 0,  OpticalFlow,    _type,   0),

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

    // the parameter description below is for GCSs (like MP) that use master for the parameter descriptions.  This should be removed when Copter-3.7 is released
    // @Param: _ENABLE
    // @DisplayName: Optical flow enable/disable
    // @Description: Setting this to Enabled(1) will enable optical flow. Setting this to Disabled(0) will disable optical flow
    // @Values: 0:Disabled, 1:Enabled
    // @User: Standard

    AP_GROUPINFO("_ORIENT_PITCH", 6,  OpticalFlow, _pitchAngle_cd,   0),

    // @Param: _ORIENT_PITCH
    // @DisplayName: Flow sensor pitch alignment
    // @Description: Specifies the number of centi-degrees that the flow sensor is pitched relative to the vehicle.
    // @Range: -18000 +18000
    // @Increment: 1
    // @User: Standard


    AP_GROUPINFO("_ORIENT_ROLL", 7,  OpticalFlow, _rollAngle_cd,   0),

    // @Param: _ORIENT_ROLL
    // @DisplayName: Flow sensor roll alignment
    // @Description: Specifies the number of centi-degrees that the flow sensor is rolled relative to the vehicle.
    // @Range: -18000 +18000
    // @Increment: 1
    // @User: Standard

    AP_GROUPEND
};

AP_OpticalFlow_Params::AP_OpticalFlow_Params(void) {
    AP_Param::setup_object_defaults(this, var_info);
}
