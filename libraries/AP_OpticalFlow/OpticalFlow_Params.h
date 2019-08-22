#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

class OpticalFlow_Params {
public:
    static const struct AP_Param::GroupInfo var_info[];

    OpticalFlow_Params(void);

    /* Do not allow copies */
    OpticalFlow_Params(const OpticalFlow_Params &other) = delete;
    OpticalFlow_Params &operator=(const OpticalFlow_Params&) = delete;

    AP_Int8  _type;
    AP_Int8  _flowScalerX;
    AP_Int8  _flowScalerY;
    AP_Int8 _yawAngle_cd;
    AP_Vector3f _pos_offset; // position offset in body frame
    AP_Int8  _address;
    AP_Int8  _pitchAngle_cd;
    AP_Int8  _rollAngle_cd;
};
