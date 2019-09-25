#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

bool start = false;
uint32_t start_ms;

// stabilize_init - initialise stabilize controller
bool Copter::ModeDelayedBuoy::init(bool ignore_checks)
{
    bool air_mode_check = false;
    for(uint8_t i=0; i<8; i++){
        if (motors->motor_medium[i] == MOTORS_AIR){
            air_mode_check = true;
            break;
        }
    }
    //if in surface or air mode and armed, pilot likely flying in air, should not buoy
    if (air_mode_check && motors->armed()){
        gcs().send_text(MAV_SEVERITY_INFO, "MODE SWITCH: delayed buoy, failed: vehicle in air state");
        return false;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "MODE SWITCH: delayed buoy");
    //set flag to require delay start upon mode switch
    start = false;
    return true;
}

void Copter::ModeDelayedBuoy::run()
{
    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !motors->get_interlock()) {
        start = false;
        return;
    }
    float temp = g2.buoy_delay;
    if (start == false){
        gcs().send_text(MAV_SEVERITY_INFO, "begin delay %f s\n", temp);
        start = true;
        start_ms = AP_HAL::millis();
    }

    uint32_t tnow_ms = AP_HAL::millis();
    for(uint8_t i = 0; i<8; i++){
        //if in delay timer, motors off, otherwise function like buoy mode
        if (tnow_ms - start_ms <= g2.buoy_delay*1000){
            motors-> motor_medium[i] = MOTORS_OFF;
        }
        else{
            motors->motor_medium[i] = MOTORS_WATER;
        }
    }
    
    copter.mode_buoy.run();
}

