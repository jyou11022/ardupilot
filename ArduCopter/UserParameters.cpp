#include "UserParameters.h"

// "USR" + 13 chars remaining for param name 
const AP_Param::GroupInfo UserParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 13 chars

    AP_GROUPINFO("_ESC_SERIAL_1", 1, UserParameters, _esc_serial_1, 0),
    AP_GROUPINFO("_ESC_SERIAL_2", 2, UserParameters, _esc_serial_2, 0),
    AP_GROUPINFO("_ESC_SERIAL_3", 3, UserParameters, _esc_serial_3, 0),
    AP_GROUPINFO("_ESC_SERIAL_4", 4, UserParameters, _esc_serial_4, 0),
    AP_GROUPINFO("_ESC_SERIAL_5", 5, UserParameters, _esc_serial_5, 0),
    AP_GROUPINFO("_ESC_SERIAL_6", 6, UserParameters, _esc_serial_6, 0),
    AP_GROUPINFO("_ESC_SERIAL_7", 7, UserParameters, _esc_serial_7, 0),
    AP_GROUPINFO("_ESC_SERIAL_8", 8, UserParameters, _esc_serial_8, 0),

    AP_GROUPEND
};
