#include "Copter.h"

/*
 * Init and run calls for Naviator Auto mode
 * 
 * This mode applies a mission profile to naviator. It is time and depth based.
 * 
 *  NaviatorCMD         - lists all possible commands
 *  MissionProfile      - struct containing mission {CMD, VALUE}
 *  nv_auto_mission     - struct array containing mission {CMD, VALUE} pairs
 *  
 *  NaviatorMedium      - fluid medium conditions of the vehicle, ie AIR/SURFACE/WATER
 *  NaviatorMotorState  - individual motor states, ie. AIR/WATER/OFF
 *  
 *  NaviatorVehicle     - struct containing vehicle state
 *  NaviatorPID         - generalized PD struct
 *  nv_auto_depth_pid   - PD depth controller struct
 *  NV_MissionState     - struct containing variables to keep track of the mission states
 *  nv_auto             - mission state struct
 * 
 *  heading_error(...)  - method to determine the heading error
 */

/* HIGH LEVEL ROUTINES */
#define NV_MODE_OFF 0
#define NV_MODE_MAINTAIN_ALTITUDE 1
#define NV_MODE_MAINTAIN_THROTTLE 2

/* LOW LEVEL ROUTINES, aka Mission Commands */
enum NaviatorCMD {
    NV_BEGIN_MISSION, // 0

    NV_SET_SUBROUTINE,            /// 0 - OFF, 1 - Maintain Alt/Depth, 2 - Maintain Throttle

    NV_WAIT_FOR_HEADING,    ///

    NV_SET_PITCH_ANGLE,     /// angle, deg
    NV_SET_YAW_RATE,        /// deg/s * 100,
    NV_SET_THROTTLE,        /// us
    NV_SET_ALTITUDE,        /// cm

    NV_RECORD_HEADING,      /// deg/s * 100,

    NV_DELAY, // ms
    NV_END_MISSION, // 0
};

/* THE MISSION */
struct MissionProfile {
    NaviatorCMD cmd;
    int32_t value;
} nv_auto_mission[] = {
    { NV_BEGIN_MISSION, 0 },

    /**/
    { NV_SET_SUBROUTINE , NV_MODE_MAINTAIN_ALTITUDE },

    // sink to -0.5m
    { NV_SET_ALTITUDE , -250 },
    { NV_SET_PITCH_ANGLE , 0 },
    { NV_DELAY , 30000 },
  
    //reset pitch 
    { NV_SET_ALTITUDE , -50 },
    { NV_SET_PITCH_ANGLE , 0 },
    { NV_DELAY , 30000 },

    // climb up
    { NV_SET_SUBROUTINE , NV_MODE_MAINTAIN_THROTTLE },

    /**/
    { NV_SET_THROTTLE , 500 },
    { NV_DELAY , 60000 * 5 },
    /**/

    { NV_SET_SUBROUTINE , NV_MODE_OFF },
    { NV_END_MISSION, 0 },
};

/* FLUID MEDIUMS */
enum NaviatorMedium {
    NV_MEDIUM_AIR,
    NV_MEDIUM_SURFACE,
    NV_MEDIUM_WATER,
    NV_MEDIUM_UNKNOWN
};

/* MOTOR STATES */
enum NaviatorMotorState {
    NV_AUTO_MOTOR_AIR,
    NV_AUTO_MOTOR_WATER,
    NV_AUTO_MOTOR_OFF
};

/* MOTOR NAVIATOR STATES */
enum NaviatorSystemState {
    NV_AUTO_SYSTEM_UNKNOWN,
    NV_AUTO_SYSTEM_AIR,
    NV_AUTO_SYSTEM_TRANSITION,
    NV_AUTO_SYSTEM_WATER
};

/* VEHICLE STATE VARIABLES */
struct NaviatorVehicle {
    int16_t target_yaw_rate     = 0;
    int16_t heading             = 0;

    int16_t target_pitch        = 0;
    int16_t target_roll         = 0;
    int16_t target_z            = 0;
    int16_t target_thr          = 0;

    int8_t medium               = NV_MEDIUM_UNKNOWN;
    int8_t motor_top_mode       = NV_AUTO_MOTOR_OFF;
    int8_t motor_bot_mode       = NV_AUTO_MOTOR_OFF;
    int8_t system_mode    = NV_AUTO_SYSTEM_UNKNOWN;

    bool initialized = false;
} nv_vehicle;

/* STANDALONE NV PID STRUCT */
#define NV_PID_D_SAMPLES 6
struct NaviatorPID {
    float nv_dz_error_buffer[NV_PID_D_SAMPLES] = { 0,0,0,0,0 , 0 };

    float nv_dz_error_old = 0;
    float nv_dz_error = 0;
    float nv_z_error = 0;
    float nv_z_output = 0;
} nv_auto_depth_pid;

/* MISSION STATE VARIABLES */
struct NV_MissionState {
    unsigned int    NV_MISSION_TOTAL = (sizeof(nv_auto_mission) / sizeof(nv_auto_mission[0]));
    unsigned int    low_level_ticker = 0;
    unsigned int    mission_index = 0;

    unsigned long   low_level_stamp = 0;

    uint8_t         high_level_subroutine = 0;
    unsigned int    high_level_ticker = 0;

    bool            pass_thr_out = true;
} nv_auto;

#define NV_NEXT_CMD nv_auto.low_level_ticker = 0; nv_auto.mission_index++; nv_auto.low_level_stamp = millis();
#define NV_LEVEL nv_vehicle.target_roll = 0; nv_vehicle.target_pitch = 0; nv_vehicle.target_yaw_rate = 0;

control_mode_t nv_orig_control_mode;

// nv_auto_init - initialise nv_auto controller
bool Copter::ModeNV_AUTO::init(bool ignore_checks)
{
    if (!ignore_checks){
        // only allow nv_auto from ACRO, Stabilize, AltHold, BUOY, DELAYED_BUOY modes
        if (copter.control_mode != ACRO &&
            copter.control_mode != STABILIZE &&
            copter.control_mode != ALT_HOLD &&
            copter.control_mode != BUOY &&
            copter.control_mode != DELAYED_BUOY) {
            gcs().send_text(MAV_SEVERITY_INFO, "MODE SWITCH: 'nv_auto' - invalid 'from' mode");
            return false;
        }

        // if in acro or stabilize ensure throttle is above zero
        if (ap.throttle_zero && (copter.control_mode == ACRO || copter.control_mode == STABILIZE)) {
            gcs().send_text(MAV_SEVERITY_INFO, "MODE SWITCH: 'nv_auto' - throttle too low");
            return false;
        }

        // ensure roll input is less than 40deg
        if (abs(channel_roll->get_control_in()) >= 4000) {
            return false;
        }

        // only allow flip when flying or swimming or on water surface
        if (!motors->armed() || ap.land_complete) {
            return false;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "MODE SWITCH: nv_auto");
    }
    else{
        gcs().send_text(MAV_SEVERITY_INFO, "MODE SWITCH: nv_auto from auto mode");
    }


    // determine current medium, and init motor states accordingly
    // for now, assuming water
    nv_vehicle.medium              = NV_MEDIUM_WATER;
    nv_vehicle.system_mode         = NV_AUTO_SYSTEM_WATER;
    nv_vehicle.motor_top_mode      = NV_AUTO_MOTOR_WATER;
    nv_vehicle.motor_bot_mode      = NV_AUTO_MOTOR_WATER;
    nv_vehicle.initialized = true;

    // reinit
    motors->motor_min_enable = false;
    nv_auto.high_level_subroutine = 0;
    NV_NEXT_CMD
    NV_LEVEL
    nv_auto.mission_index = 0;

    // capture original flight mode so that we can return to it after completion
    nv_orig_control_mode = copter.control_mode;

    return true;
}

//auto mode gives cmd with parameters to determine the mission action
void Copter::ModeNV_AUTO::do_command(const AP_Mission::Mission_Command& cmd){
    if (!nv_vehicle.initialized) init(true);
    current_cmd = cmd;
    switch(cmd.id){
        case AP_MISSION_UW_THROTTLE:
            // reset the high level subroutine ticker
            nv_auto.high_level_ticker = 0;
            // select the high level subroutine
            nv_auto.high_level_subroutine = NV_MODE_MAINTAIN_THROTTLE;

            gcs().send_text(MAV_SEVERITY_INFO, "NV_SET_THROTTLE");

            // MAYBE TBA: check if in "NV_MODE_MAINTAIN_THROTTLE" HL_SUBROUTINE and if not, set it
            // for now, I prefer for these commands not to interfere with each other, but later, we
            // may want the redundancy in case the mission is entered incorrectly.
            motors->motor_min_enable = true;
            // update the target throttle of the vehicle, constrained by 0..1000
            nv_vehicle.target_thr = constrain_int16(cmd.content.uw_throttle.target_throttle*1000, 0, 1000);
            // report target throttle command value
            gcs().send_text(MAV_SEVERITY_INFO, " - - thr< %d", (nv_vehicle.target_thr));
            break;
        case AP_MISSION_UW_ALTITUDE:
            gcs().send_text(MAV_SEVERITY_INFO, "NV_SET_SUBROUTINE");
            
            // reset the high level subroutine ticker
            nv_auto.high_level_ticker = 0;
            // select the high level subroutine
            nv_auto.high_level_subroutine = NV_MODE_MAINTAIN_ALTITUDE;

            // update the target z position of the vehicle
            nv_vehicle.target_z = cmd.content.uw_altitude.target_altitude;

            cr_fs_check = false;
            cr_fs_check_start = AP_HAL::millis();
            break;

        case AP_MISSION_UW_ATTITUDE:
            nv_vehicle.target_roll = cmd.content.uw_attitude.roll;
            nv_vehicle.target_pitch = cmd.content.uw_attitude.pitch;
            nv_vehicle.target_yaw_rate = cmd.content.uw_attitude.yaw_rate;
            gcs().send_text(MAV_SEVERITY_INFO, "NV_SET_ATTITUDE: roll %d, pitch %d, yaw_rate %d", nv_vehicle.target_roll, nv_vehicle.target_pitch, nv_vehicle.target_yaw_rate);
            break;

        default:
            break;
    }

    att_fs_check_start = AP_HAL::millis();
    att_fs_check = false;
}

bool Copter::ModeNV_AUTO::verify(){
    switch(current_cmd.id){
        case AP_MISSION_UW_THROTTLE:
            return true;
            break;
        case AP_MISSION_UW_ALTITUDE:
            // float sign;
            // if (nv_vehicle.target_z - copter.baro_alt < 0) sign = 1;
            // else sign = -1;

            // if (copter.barometer.get_climb_rate()*sign < 0.1){
            //     if (!cr_fs_check){
            //         cr_fs_check = true;
            //         cr_fs_check_start = AP_HAL::millis();
            //     }
            //     else if (AP_HAL::millis() - cr_fs_check_start > g2.cr_fs_time_s * 1000){
            //         gcs().send_text(MAV_SEVERITY_ERROR, "NV_UW_ALTITUDE: vehicle not climbing/sinking properly");
            //         set_mode(LAND, MODE_REASON_TERMINATE);
            //     }
            // }

            return fabs(nv_vehicle.target_z - copter.baro_alt) < 5;
            break;
        case AP_MISSION_UW_ATTITUDE:
            return (fabs(nv_vehicle.target_roll*100 - ahrs.roll_sensor) < 500 &&
                fabs(nv_vehicle.target_pitch*100 - ahrs.pitch_sensor) < 500);
            break;
        default:
            break;
    }
    return false;
}

// nv_auto_run - runs the main nv_auto controller
// should be called at 100hz or more
void Copter::ModeNV_AUTO::run()
{
    /// PROCESS MISSION
    if (nv_auto.mission_index < nv_auto.NV_MISSION_TOTAL)
    {
        process_mission();
        run_mission();
    }
    /// MISSION COMPLETE
    else {
        if (nv_auto.mission_index == nv_auto.NV_MISSION_TOTAL) {
            gcs().send_text(MAV_SEVERITY_INFO, "NV_AUTO: COMPLETE"); // inform the GCS
            nv_auto.mission_index++;
        }
        if (nv_auto.low_level_ticker == 0) {
            // initialize vertical speeds and leash lengths
            attitude_control->set_throttle_out(0, false, g.throttle_filt);

            // return to previous mode
            if (!copter.set_mode(nv_orig_control_mode, MODE_REASON_MISSION_END)) {
                // this should never happen but just in case
                copter.set_mode(STABILIZE, MODE_REASON_UNKNOWN);
            }

            //init_disarm_motors();
            nv_auto.low_level_ticker++;
        }
    }
}

void Copter::ModeNV_AUTO::process_mission(){
    int32_t yaw_angle = ahrs.yaw_sensor;
    /* PROCESS COMMANDS */
        // run through all instant commands
        // and break when it is a time-dependent command
    while (nv_auto.mission_index < nv_auto.NV_MISSION_TOTAL)
    {
        // run NV_BEGIN_MISSION
        if (nv_auto_mission[nv_auto.mission_index].cmd == NV_BEGIN_MISSION) {
            // simply let the GCS know we're starting a mission
            gcs().send_text(MAV_SEVERITY_INFO, "DATA_NV_AUTO_BEGIN");
            // also, report how much memory we have available so we keep an eye on mission size
            gcs().send_text(MAV_SEVERITY_INFO, "MEM %u", static_cast<uint32_t>((unsigned)hal.util->available_memory()));

            NV_NEXT_CMD
        }
        // run NV_SET_ALTITUDE
        else if (nv_auto_mission[nv_auto.mission_index].cmd == NV_SET_ALTITUDE) {
            gcs().send_text(MAV_SEVERITY_INFO, "NV_SET_ALTITUDE");
            
            // update the target z position of the vehicle
            nv_vehicle.target_z = nv_auto_mission[nv_auto.mission_index].value;

            NV_NEXT_CMD
        }
        // run NV_SET_SUBROUTINE
        else if (nv_auto_mission[nv_auto.mission_index].cmd == NV_SET_SUBROUTINE) {
            gcs().send_text(MAV_SEVERITY_INFO, "NV_SET_SUBROUTINE");
            
            // reset the high level subroutine ticker
            nv_auto.high_level_ticker = 0;
            // select the high level subroutine
            nv_auto.high_level_subroutine = nv_auto_mission[nv_auto.mission_index].value;
            
            NV_NEXT_CMD
        }
        // run NV_SET_PITCH_ANGLE
        else if (nv_auto_mission[nv_auto.mission_index].cmd == NV_SET_PITCH_ANGLE) {
            gcs().send_text(MAV_SEVERITY_INFO, "NV_SET_PITCH_ANGLE");
            
            // update the target pitch of the vehicle
            nv_vehicle.target_pitch = nv_auto_mission[nv_auto.mission_index].value;
            
            NV_NEXT_CMD
        }
        // run NV_SET_YAW_RATE
        else if (nv_auto_mission[nv_auto.mission_index].cmd == NV_SET_YAW_RATE) {
            gcs().send_text(MAV_SEVERITY_INFO, "NV_SET_YAW_RATE");
            
            // update the target yaw rate of the vehicle
            nv_vehicle.target_yaw_rate = nv_auto_mission[nv_auto.mission_index].value;
            
            NV_NEXT_CMD
        }
        // run NV_RECORD_HEADING
        else if (nv_auto_mission[nv_auto.mission_index].cmd == NV_RECORD_HEADING) {
            gcs().send_text(MAV_SEVERITY_INFO, "NV_RECORD_HEADING");

            // record and report the current heading of the vehicle
            nv_vehicle.heading = (int16_t)(yaw_angle / 100);
            gcs().send_text(MAV_SEVERITY_INFO, " - - head< %u", static_cast<uint32_t>(nv_vehicle.heading));

            NV_NEXT_CMD
        }
        // run NV_SET_THROTTLE
        else if (nv_auto_mission[nv_auto.mission_index].cmd == NV_SET_THROTTLE) {
            if (nv_auto.low_level_ticker == 0) {
                gcs().send_text(MAV_SEVERITY_INFO, "NV_SET_THROTTLE");

                // MAYBE TBA: check if in "NV_MODE_MAINTAIN_THROTTLE" HL_SUBROUTINE and if not, set it
                // for now, I prefer for these commands not to interfere with each other, but later, we
                // may want the redundancy in case the mission is entered incorrectly.

                nv_auto.low_level_ticker++;
                motors->motor_min_enable = true;
            }
            else {
                // update the target throttle of the vehicle, constrained by 0..1000
                nv_vehicle.target_thr = constrain_int16(nv_auto_mission[nv_auto.mission_index].value, 0, 1000);
                // report target throttle command value
                gcs().send_text(MAV_SEVERITY_INFO, " - - thr< %u", static_cast<uint32_t>(nv_vehicle.target_thr));

                NV_NEXT_CMD
            }
        }
        // run NV_DELAY
        else if (nv_auto_mission[nv_auto.mission_index].cmd == NV_DELAY) {
            if (nv_auto.low_level_ticker == 0) {
                gcs().send_text(MAV_SEVERITY_INFO, "NV_DELAY");
                nv_auto.low_level_ticker++;
            }
            else {
                // only execute next command if "time since last cmd reset" > "desired delay"
                if (millis() - nv_auto.low_level_stamp > nv_auto_mission[nv_auto.mission_index].value) {
                    NV_NEXT_CMD
                }
            }
            break;
        }
        // run NV_WAIT_FOR_HEADING
        else if (nv_auto_mission[nv_auto.mission_index].cmd == NV_WAIT_FOR_HEADING) {
            if (nv_auto.low_level_ticker == 0) {
                gcs().send_text(MAV_SEVERITY_INFO, "NV_WAIT_FOR_HEADING");
                nv_auto.low_level_ticker++;
            }

            // determine direction
            uint8_t nv_dir;
            if (nv_vehicle.target_yaw_rate < 0) nv_dir = 1; else nv_dir = 0;

            // calculate the heading error, in degrees
            uint16_t nv_h_error = heading_error((int32_t)(yaw_angle / 100), nv_vehicle.heading, nv_dir);

            // if the error is less than 1 degree, then continue to the next cmd
            if (nv_h_error < 1 || nv_h_error > 359) {
                NV_NEXT_CMD
            }
            break;
        }
        // run DATA_NV_AUTO_END
        else if (nv_auto_mission[nv_auto.mission_index].cmd == NV_END_MISSION) {
            gcs().send_text(MAV_SEVERITY_INFO, "DATA_NV_AUTO_END");
            // do something at the end of the mission (ie. surface, go to mode, etc...)
            // ...nothing to see here, for now
            NV_NEXT_CMD
        }
        // unrecognized command
        else {
            NV_NEXT_CMD
        }
    }
}

void Copter::ModeNV_AUTO::run_mission(){

    if (!motors->armed()){
        att_fs_check = false;
        cr_fs_check = false;
    }

    //for now all missions are in water mode
    for(uint8_t i = 0; i<8; i++){
        motors->motor_medium[i] = MOTORS_WATER;
    }

    /* local variables */
    float target_roll = 0, target_pitch = 0;
    float target_yaw_rate = 0;
    
    // get current angles, degrees * 100
    //int32_t pitch_angle = ahrs.pitch_sensor;
    //int32_t roll_angle = ahrs.roll_sensor;

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
    update_simple_mode();

    // convert pilot input to lean angles, will need to add a parameter for underwater parameters
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, copter.aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll + nv_vehicle.target_roll * 100, target_pitch + nv_vehicle.target_pitch * 100, target_yaw_rate + nv_vehicle.target_yaw_rate);

    //attitude failsafe checker
    if ((fabs(nv_vehicle.target_roll*100 - ahrs.roll_sensor) > 500 ||
        fabs(nv_vehicle.target_pitch*100 - ahrs.pitch_sensor) > 500)){
        if (!att_fs_check){
            att_fs_check = true;
            att_fs_check_start = AP_HAL::millis();
        }
        else
        if (AP_HAL::millis() - att_fs_check_start > g2.att_fs_time_s * 1000){
            gcs().send_text(MAV_SEVERITY_ERROR, "NV_UW_ATTITUDE: vehicle missed attitude deadline");
            att_fs_check = false;
            set_mode(LAND, MODE_REASON_EKF_FAILSAFE);
        }
    }else att_fs_check = false;

    /* PROCESS MODE */
    if (nv_auto.high_level_subroutine == NV_MODE_MAINTAIN_ALTITUDE) {
        // NOTE: technically, we need to check if we are actually in the air or water
        // but for now, assume water

        // STEP 1: If we are just starting this high level subroutine, then determine which
        // fluid medium we are in and report back to GCS
        if (nv_auto.high_level_ticker == 0) {
            // let GCS know we are starting the altitude/depth hold high level subroutine
            gcs().send_text(MAV_SEVERITY_INFO, "NV_MAINTAIN_ALTITUDE Start");
            // give some initial condition details
            gcs().send_text(MAV_SEVERITY_INFO, " - - alt: %d -> %d",
                                            static_cast<uint32_t>(copter.baro_alt),
                                            static_cast<uint32_t>(nv_vehicle.target_z));

            // next, we determine which fluid medium the vehicle is in and intialize the corresponding controller

            // if in water
            // (for now, we are assuming this to be true and so, the assumption
            // must ensure somewhere else that NV_AUTO always results in water mode for motors)
            if (true) {
                gcs().send_text(MAV_SEVERITY_INFO, " - - (in water)");

                // initialize the depth hold controller
                nv_auto_depth_pid.nv_z_error = (float)nv_vehicle.target_z - copter.baro_alt;
                nv_auto_depth_pid.nv_dz_error = 0;

                // for future tracking purposes, update the vehicle state to reflect being in water
                nv_vehicle.medium              = NV_MEDIUM_WATER;
                nv_vehicle.system_mode         = NV_AUTO_SYSTEM_WATER;
                nv_vehicle.motor_top_mode      = NV_AUTO_MOTOR_WATER;
                nv_vehicle.motor_bot_mode      = NV_AUTO_MOTOR_WATER;

                // for throttle control, this might be useful, but here,
                // sometimes the only way to descend is to cut throttle completely
                motors->motor_min_enable = false;

                // the water controller passes the throttle directly
                nv_auto.pass_thr_out = true;
                
                // route the subroutine to the (1) water high level control loop
                nv_auto.high_level_ticker = 1;
            }
            // if in air
            else if(false) {
                gcs().send_text(MAV_SEVERITY_INFO, " - - (in air)");

                // for future tracking purposes, update the vehicle state to reflect being in air
                nv_vehicle.medium              = NV_MEDIUM_AIR;
                nv_vehicle.system_mode         = NV_AUTO_SYSTEM_AIR;
                nv_vehicle.motor_top_mode      = NV_AUTO_MOTOR_AIR;
                nv_vehicle.motor_bot_mode      = NV_AUTO_MOTOR_AIR;
                
                // the althold controller DOES NOT pass the throttle directly
                nv_auto.pass_thr_out = false;

                /* ------------------------------------------------------------------------- */
                // TBA: put in the altitude hold controller initialization here...
                /* ------------------------------------------------------------------------- */

                // route the subroutine to the (2) air high level control loop
                nv_auto.high_level_ticker = 2;
            }
            // if in surface
            else if(false) {
                gcs().send_text(MAV_SEVERITY_INFO, " - - (at surface)");
                
                // for future tracking purposes, update the vehicle state to reflect being at the surface
                nv_vehicle.medium              = NV_MEDIUM_SURFACE;
                nv_vehicle.system_mode         = NV_AUTO_SYSTEM_TRANSITION;
                nv_vehicle.motor_top_mode      = NV_AUTO_MOTOR_AIR;
                nv_vehicle.motor_bot_mode      = NV_AUTO_MOTOR_WATER;

                // the surface controller passes the throttle directly
                nv_auto.pass_thr_out = true;

                // assuming water, alt-hold initialization in air TBA...

                // route the subroutine to the (3) surface high level control loop
                nv_auto.high_level_ticker = 3;
            }
            // if unknown medium
            else {
                // could not determine fluid medium...
                // TBA: trigger failsafe...for now, do nothing...
            }
        }

        // run (high level control loop) water
        if (nv_auto.high_level_ticker == 1) {

            /* PID_Z calculation */
            float nv_z_error = nv_vehicle.target_z - copter.baro_alt;
            float nv_error_diff = (nv_z_error - nv_auto_depth_pid.nv_z_error);
            float nv_error_diff_sum = 0;
            // shift the buffer for the average filter
            for (int nv_i = 0; nv_i < (NV_PID_D_SAMPLES - 1); nv_i++) {
                nv_auto_depth_pid.nv_dz_error_buffer[nv_i] = nv_auto_depth_pid.nv_dz_error_buffer[nv_i + 1];
                nv_error_diff_sum += nv_auto_depth_pid.nv_dz_error_buffer[nv_i + 1];
            }
            // add current iteration to buffer and update the sum
            nv_auto_depth_pid.nv_dz_error_buffer[NV_PID_D_SAMPLES - 1] = nv_error_diff;
            nv_error_diff_sum += nv_error_diff;
            
            nv_auto_depth_pid.nv_z_error = nv_z_error;
            nv_auto_depth_pid.nv_dz_error = nv_error_diff_sum / (float)NV_PID_D_SAMPLES / G_Dt;
            nv_auto_depth_pid.nv_dz_error_old = nv_error_diff;
            //
            nv_auto_depth_pid.nv_z_output = (g2.nv_ap_pid_z_kP*nv_auto_depth_pid.nv_z_error + g2.nv_ap_pid_z_kD*nv_auto_depth_pid.nv_dz_error);

            // compensate for gravity
            float nv_throttle_out = nv_auto_depth_pid.nv_z_output + g2.nv_ap_pid_z_C;

            // compensate for pitch
            if (ahrs.cos_pitch() > 0)
                nv_throttle_out = nv_throttle_out / ahrs.cos_pitch();
            else
                nv_throttle_out = 1000;

            // update throttle with throttle compensation, nv_ap_pid_z_C
            nv_vehicle.target_thr = constrain_int16((int16_t)nv_throttle_out, 0, 1000);

            //climb rate failsafe checker
            float sign;
            if (nv_z_error > 0) sign = 1;
            else sign = -1;
            if (fabs(nv_z_error) > 20 && copter.barometer.get_climb_rate() * sign < 0.1){
                if (!cr_fs_check){
                    cr_fs_check = true;
                    cr_fs_check_start = AP_HAL::millis();
                }
                else
                if (AP_HAL::millis() - cr_fs_check_start > g2.cr_fs_time_s * 1000){
                    gcs().send_text(MAV_SEVERITY_ERROR, "NV_UW_ALTITUDE: vehicle missed altitude deadline");
                    set_mode(LAND, MODE_REASON_TERMINATE);
                    cr_fs_check = false;
                }
            } else cr_fs_check = false;
        }
        // run (high level control loop) air 
        else if (nv_auto.high_level_ticker == 2) {
            //
            // assuming water, alt-hold in air TBA...
            //
        }
        // run (high level control loop) surface
        else if (nv_auto.high_level_ticker == 3) {
            //
            // assuming water, dealing with surface condition TBA...
            //
        }
        // run (high level control loop) unknown
        else {
            // something is wrong...enable pass-through throttle and cut thottle
            nv_auto.pass_thr_out = true;
            nv_vehicle.target_thr = 0;
        }

    }
    else if (nv_auto.high_level_subroutine == NV_MODE_MAINTAIN_THROTTLE) {
        if (nv_auto.high_level_ticker == 0) {
            nv_auto.pass_thr_out = true;
            nv_auto.high_level_ticker++;
        }
    }
    else {
        // something is wrong...enable pass-through throttle and cut thottle
        nv_auto.pass_thr_out = true;
        nv_vehicle.target_thr = 0;
    }

    /* POST PROCESS */
    if (nv_auto.pass_thr_out) attitude_control->set_throttle_out((float)nv_vehicle.target_thr/1000.0f, false, g.throttle_filt);
}

int32_t Copter::ModeNV_AUTO::heading_error(int32_t current, int32_t target, uint8_t dir) {
    //// dir
    // 0 = CW
    // 1 = CCW
    // 2 = minimum error
    
    int32_t error_ccw = 0;
    int32_t error_cw = 0;
    int32_t error = 0;

    // ie. 90 going to 340
    if (target > current)
        error_ccw = 360 - target + current;
    else
        error_ccw = current - target;

    error_cw = 360 - error_ccw;

    if (dir == 0) error = error_cw;
    else if(dir == 1) error = error_ccw;
    else if(error_cw < error_ccw) error = error_cw;
    else error = error_ccw;

    if (error == 360) error = 0;

    return error;
}
