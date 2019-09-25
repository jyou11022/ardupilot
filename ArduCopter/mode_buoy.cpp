#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::ModeBuoy::init(bool ignore_checks)
{
    inflatable_deployed = false;
    not_climbing_maybe = false;
    bool air_mode_check = false;
    for(uint8_t i=0; i<8; i++){
        if (motors->motor_medium[i] == MOTORS_AIR){
            air_mode_check = true;
            break;
        }
    }
    if (air_mode_check && motors->armed()){
        gcs().send_text(MAV_SEVERITY_INFO, "MODE SWITCH: buoy, failed because vehicle in air state");
        return false;
    }
    gcs().send_text(MAV_SEVERITY_INFO, "MODE SWITCH: buoy");

    uart = AP_SerialManager::get_instance()->find_serial(AP_SerialManager::SerialProtocol_Feather);
    if (uart == nullptr){
        gcs().send_text(MAV_SEVERITY_INFO, "Feather uart driver not found");
    }
    return true;
}

//based on the run function from stabilize
void Copter::ModeBuoy::run()
{
    if (!motors->armed() || !motors->get_interlock()){
        for(uint8_t i=0; i<8; i++){
            motors->motor_medium[i] = MOTORS_OFF;
        }
        not_climbing_maybe = false;
        inflatable_deployed = false;
        return;
    }

    //if inflatable_deployed, kill motors and do nothing
    if (inflatable_deployed){
        for(uint8_t i=0; i<8; i++){
            motors->motor_medium[i] = MOTORS_OFF;
        }
        if (uart == nullptr){
            uart = AP_SerialManager::get_instance()->find_serial(AP_SerialManager::SerialProtocol_Feather);
        }
        if (uart != nullptr){
            uart->print("i");
        }
        return;
    }

    //CURRENTLY NOT WORKING
    //check for ESC health, deploy inflatable if any is unhealthy
    // uint8_t esc_index = motors->ESC_unhealthy();
    // if (esc_index > 0 && motors->get_throttle() > 0.0f){
    //     gcs().send_text(MAV_SEVERITY_ERROR, "ESC %d not responding, deploying inflatable", esc_index);
    //     inflatable_deployed = true;
    //     return;
    // }

    //check for vehicle on surface through sensors, and if expecting radio check if vehicle on surface by seeing if radio has signal
    if (copter.baro_alt > -15 && fabs(copter.barometer.get_climb_rate()) < 0.1f && 
        (!g.failsafe_throttle || (g.failsafe_throttle && !copter.failsafe.radio))){
        not_climbing_maybe = false;
        for(uint8_t i=0; i<4; i++){
            motors->motor_medium[i] = MOTORS_OFF;
        }
        for(uint8_t i=4; i<8; i++){
            motors->motor_medium[i] = MOTORS_WATER;
        }
    }
    //vehicle not on surface
    else{
        //if vehicle is not climbing but throttle is outputting, start timer
        if (copter.barometer.get_climb_rate() < 0.1f && motors->get_throttle() > 0.0f){
            if (!not_climbing_maybe){
                not_climbing_maybe = true;
                not_climb_start = AP_HAL::millis();
            }
            //if vehicle not climbing for enough time, expects radio and doesn't have radio, deploy float
            else if (AP_HAL::millis() - not_climb_start > g2.cr_fs_time_s * 1000 && g.failsafe_throttle && copter.failsafe.radio){
                inflatable_deployed = true;
                gcs().send_text(MAV_SEVERITY_ERROR, "BUOY: Vehicle not climbing, deploying inflatable");
                return;
            }
        }
        else{
            not_climbing_maybe = false;
        }

        for(uint8_t i = 0; i<8; i++){
            motors->motor_medium[i] = MOTORS_WATER;
        }
    }

    float target_throttle = g2.buoy_throttle * 1000;

    float target_roll, target_pitch;
    float target_yaw_rate;
    float pilot_throttle_scaled;

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

//    AP_Vehicle::MultiCopter &aparm = copter.aparm;

    // convert pilot input to lean angles
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // set constant throttle to rise copter to surface
    pilot_throttle_scaled = get_pilot_desired_throttle(target_throttle);

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}

