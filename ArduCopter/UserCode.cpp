#include "Copter.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here

    //update user's control method
    uint16_t radio_ch8 = hal.rcin->read(7);
    if(radio_ch8 > 900 && radio_ch8 < 1350 && cur_motor_control_mode != WATER) {
        cur_motor_control_mode = WATER;
        gcs().send_text(MAV_SEVERITY_INFO, "Vehicle control: Underwater");
    }
    else if(radio_ch8 > 1375 && radio_ch8 < 1750 && cur_motor_control_mode != SURFACE) {
        cur_motor_control_mode = SURFACE;
        gcs().send_text(MAV_SEVERITY_INFO, "Vehicle control: Surface");
    }
    else if(radio_ch8 > 1750 && radio_ch8 < 2100 && cur_motor_control_mode != AIR) {
        cur_motor_control_mode = AIR;
        gcs().send_text(MAV_SEVERITY_INFO, "Vehicle control: Air");
    }

    //process if motors are in air or water
    if (!top_in_water && motors->top_is_underwater()){
        top_in_water = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Top motors underwater");
    }
    if (top_in_water && !motors->top_is_underwater()){
        top_in_water = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Top motors air");
    }
    if (!bot_in_water && motors->bot_is_underwater()){
        bot_in_water = true;
        gcs().send_text(MAV_SEVERITY_INFO, "Bot motors underwater");
    }
    if (bot_in_water && !motors->bot_is_underwater()){
        bot_in_water = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Bot motors air");
    }

    bool all_underwater = true;
    for(uint8_t i=0; i<8; i++){
        if (motors->motor_medium[i] != MOTORS_WATER){
            all_underwater = false;
            break;
        }
    }
    if (all_underwater && control_mode != TRANSITION){
        aparm.angle_max = g2.water_angle_max;
    }
    else{
        aparm.angle_max = g2.air_angle_max;
    }

    //TODO: reimplement motor_medium controllers to their respective flight modes
    if (control_mode == NV_AUTO){
        // top
        if(motors->motor_medium[3] != MOTORS_WATER) {
            for (uint8_t mi = 0; mi < 4; mi++)
            {
                motors->motor_medium[mi] = MOTORS_WATER;
            }
        }
        // bot
        if(motors->motor_medium[7] != MOTORS_WATER) {
            for (uint8_t mi = 4; mi < 8; mi++)
            {
                motors->motor_medium[mi] = MOTORS_WATER;
            }
        }
    }
    else if (control_mode == STABILIZE || control_mode == ALT_HOLD || control_mode == ACRO){

        // update motor medium based on water sensor state
        if(cur_motor_control_mode == WATER) {
            // top
            if(motors->motor_medium[3] != MOTORS_WATER) {
                for (uint8_t mi = 0; mi < 4; mi++)
                {
                    motors->motor_medium[mi] = MOTORS_WATER;
                }
            }
            // bot
            if(motors->motor_medium[7] != MOTORS_WATER) {
                for (uint8_t mi = 4; mi < 8; mi++)
                {
                    motors->motor_medium[mi] = MOTORS_WATER;
                }
            }
        }
        else if(cur_motor_control_mode == SURFACE) {
            // top
            if(motors->motor_medium[3] != MOTORS_AIR) {
                for (uint8_t mi = 0; mi < 4; mi++)
                {
                    motors->motor_medium[mi] = MOTORS_AIR;
                }
            }
            // bot
            if(motors->motor_medium[7] != MOTORS_WATER) {
                for (uint8_t mi = 4; mi < 8; mi++)
                {
                    motors->motor_medium[mi] = MOTORS_WATER;
                }
            }
            aparm.angle_max = g2.air_angle_max;
        }
        else {
            // top
            if(motors->motor_medium[3] != MOTORS_AIR) {
                for (uint8_t mi = 0; mi < 4; mi++)
                {
                    motors->motor_medium[mi] = MOTORS_AIR;
                }
            }
            // bot
            if(motors->motor_medium[7] != MOTORS_AIR) {
                for (uint8_t mi = 4; mi < 8; mi++)
                {
                    motors->motor_medium[mi] = MOTORS_AIR;
                }
            }
            aparm.angle_max = g2.air_angle_max;
        }
    }   
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
