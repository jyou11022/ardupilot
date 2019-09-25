#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters() {}
    static const struct AP_Param::GroupInfo var_info[];
    
    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Int32 get_escSerial1Param() const { return _esc_serial_1; }
    AP_Int32 get_escSerial2Param() const { return _esc_serial_2; }
    AP_Int32 get_escSerial3Param() const { return _esc_serial_3; }
    AP_Int32 get_escSerial4Param() const { return _esc_serial_4; }
    AP_Int32 get_escSerial5Param() const { return _esc_serial_5; }
    AP_Int32 get_escSerial6Param() const { return _esc_serial_6; }
    AP_Int32 get_escSerial7Param() const { return _esc_serial_7; }
    AP_Int32 get_escSerial8Param() const { return _esc_serial_8; }
 
private:
    // Put your parameter variable definitions here
    AP_Int32 _esc_serial_1;
    AP_Int32 _esc_serial_2;
    AP_Int32 _esc_serial_3;
    AP_Int32 _esc_serial_4;
    AP_Int32 _esc_serial_5;
    AP_Int32 _esc_serial_6;
    AP_Int32 _esc_serial_7;
    AP_Int32 _esc_serial_8;
};
