#include "Copter.h"

bool Copter::ModeTransition::init(bool ignore_checks)
{
    _hover_height = g2.transition_height;
    _state = UNDERWATER;

    if (!ignore_checks && motors->armed() && !motors->is_underwater()){
        gcs().send_text(MAV_SEVERITY_INFO, "MODE SWITCH: transition failed because armed and in air");
        return false;
    }

    gcs().send_text(MAV_SEVERITY_INFO, "MODE SWITCH:transition-height: %fm, angle:%fdegrees", (float)_hover_height, (float)g2.trans_max_angle/100.0f);
    return true;
}

//auto mode calls this function to give user specified parameters in Mission_Command struct
void Copter::ModeTransition::do_command(const AP_Mission::Mission_Command& cmd){
    init(true);
    gcs().send_text(MAV_SEVERITY_INFO, "auto mission override of hover height: %f", cmd.content.transition.height);
    //overwrite ardupilot height parameter with specific command height parameter
    _hover_height = cmd.content.transition.height;
}

void Copter::ModeTransition::run(){

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        _state = UNDERWATER;
        return;
    }

    //state logic machine
    switch(_state){
        default:
            if (motors->is_underwater()){
                gcs().send_text(MAV_SEVERITY_INFO, "Vehicle hit water");
                _state = UNDERWATER;
            }
            break;
        case UNDERWATER:
            //wait for vehicle to be around surface depth and inav agrees or converges with barometer readings
            //also wait for vehicle to be righted
            if (fabs(copter.baro_alt) < 25.0f
                && degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch())) * 100.0f < (float)g2.trans_max_angle){
                gcs().send_text(MAV_SEVERITY_INFO, "Vehicle near surface, switching to closed loop control");
                _state = SURFACE;
            }
            break;
        case SURFACE:
            //wait for top motors to exit water and vehicle is righted
            if (!motors->top_is_underwater() && degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch())) * 100.0f < (float)g2.trans_max_angle){
                gcs().send_text(MAV_SEVERITY_INFO, "Vehicle righted on surface, transitioning, aiming for %f cmps", (float)(g2.trans_goalrt));

                air_init();
                _state = TRANSITION_STATE;
            }
            break;
        case TRANSITION_STATE:
            //wait for vehicle to fully exit water. verifies this by seeing if vehicle has achieved a high enough climb rate
            if (!motors->is_underwater() && copter.climb_rate >= g2.trans_goalrt){
                gcs().send_text(MAV_SEVERITY_INFO, "Vehicle fully airborne, climbing for %f alt at %f cmps", (float)_hover_height, (float)g2.trans_climbrt);
                _state = AIR_CLIMB;
            }
            break;
        case AIR_CLIMB:
            //if vehicle has hit/exceeded target altitude, just hover
            if (inertial_nav.get_altitude() >= _hover_height*100){
                gcs().send_text(MAV_SEVERITY_INFO, "Vehicle attained goal altitude %f, holding", (float)_hover_height);
                _state = AIR_HOLD;
            }
            //if vehicle has reentered the water for whatever reason, reset and try to transition out again
            if (motors->is_underwater()){
                gcs().send_text(MAV_SEVERITY_INFO, "Vehicle hit water");
                _state = UNDERWATER;
            }
            break;
        case AIR_HOLD:
            //if vehicle has dropped below target altitude by a meter, try to climb back to target altitude
            if (inertial_nav.get_altitude() < _hover_height*100 - 100.0f){
                gcs().send_text(MAV_SEVERITY_INFO, "Vehicle dropped below %f, climbing", _hover_height - 1.0f);
                _state = AIR_CLIMB;
            }
            //if vehicle has reentered water for whatever reason, reset and try to transition out again
            if (motors->is_underwater()){
                gcs().send_text(MAV_SEVERITY_INFO, "Vehicle hit water");
                _state = UNDERWATER;
            }
            break;
    }

    //state behavior machine
    switch(_state){
        default:
            break;
        //underwater the vehicle should try to surface in open loop similar to buoy mode
        case UNDERWATER:
            for(uint8_t i=0; i<8; i++){
                motors->motor_medium[i] = MOTORS_WATER;
            }
            underwater_run(g2.buoy_throttle);
            break;
        //on surface the vehicle should be in closed loop to be ready for transition but low throttle to not transition
        case SURFACE:
            for(uint8_t i=0; i<8; i++){
                motors->motor_medium[i] = MOTORS_AIR;
            }
            underwater_run(0.2f);
            break;
        //transitioning the vehicle should be using the althold controller but trying to exit the water asap
        case TRANSITION_STATE:
            for(uint8_t i=0; i<8; i++){
                motors->motor_medium[i] = MOTORS_AIR;
            }
            air_run(g.pilot_speed_up/2.0f);
            break;
        //climbing in air should just be althold mode climbing
        case AIR_CLIMB:
            for(uint8_t i=0; i<8; i++){
                motors->motor_medium[i] = MOTORS_AIR;
            }
            air_run(g2.trans_climbrt);
            break;
        //hovering in air should just be althold mode hovering
        case AIR_HOLD:
            for(uint8_t i=0; i<8; i++){
                motors->motor_medium[i] = MOTORS_AIR;
            }
            air_run(0.0f);
            break;
    }
}

// should be called at 100hz or more
// underwater behavior, just runs stabilize loop with parameter determined throttle 
void Copter::ModeTransition::underwater_run(float throttle)
{
    float target_roll, target_pitch;
    float target_yaw_rate;

    // clear landing flag
    set_land_complete(false);

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    //AP_Vehicle::MultiCopter &aparm = copter.aparm;

    // convert pilot input to lean angles
    //get_pilot_desired_lean_angles(target_roll, target_pitch, aparm.angle_max, aparm.angle_max);
    target_roll = 0;
    target_pitch = 0;

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control->set_throttle_out(throttle, true, g.throttle_filt);
}

// althold_init - initialise althold controller
bool Copter::ModeTransition::air_init()
{
    // initialise position and desired velocity
    if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
        //pos_control->shift_alt_target(_hover_height * 100);
        pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }

    return true;
}

// should be called at 100hz or more
// air behavior, just runs althold loop with function argument determined climb or hold
void Copter::ModeTransition::air_run(float climb_rate)
{
    //takeoff gives extra throttle to exit ground effect and water, so transition should use takeoff state
    //any other use of althold mode should be just flying
    AltHoldModeState althold_state = 
        //(_state == TRANSITION_STATE && motors->is_underwater()) ? AltHold_Takeoff : 
        AltHold_Flying;
    float takeoff_climb_rate = 0.0f;

    // initialize vertical speeds and acceleration
    pos_control->set_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control->set_accel_z(g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = constrain_float(g2.trans_climbrt, -get_pilot_speed_dn(), g.pilot_speed_up);
    target_climb_rate = climb_rate;

    // Alt Hold State Machine Determination
/*
    if (!motors->armed() || !motors->get_interlock()) {
        althold_state = AltHold_MotorStopped;
    } else if (takeoff.running() || takeoff.triggered(target_climb_rate)) {
        althold_state = AltHold_Takeoff;
    } else if (!ap.auto_armed || ap.land_complete) {
        althold_state = AltHold_Landed;
    } else {
        althold_state = AltHold_Flying;
    }
*/

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:

        motors->set_desired_spool_state(AP_Motors::DESIRED_SHUT_DOWN);
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#if FRAME_CONFIG == HELI_FRAME    
        // force descent rate and call position controller
        pos_control->set_alt_target_from_climb_rate(-abs(g.land_speed), G_Dt, false);
        heli_flags.init_targets_on_arming=true;

        if (ap.land_complete_maybe) {
            pos_control->relax_alt_hold_controllers(0.0f);
        }
#else
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
#endif
        pos_control->update_z_controller();
        break;

    case AltHold_Takeoff:
#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            heli_flags.init_targets_on_arming=false;
        }
#endif
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
            // indicate we are taking off
            set_land_complete(false);
            // clear i terms
            set_throttle_takeoff();
            //pos_control->set_desired_velocity_z(g2.trans_goalrt);
        }

        target_climb_rate = g.pilot_speed_up;

        // get take-off adjusted pilot and takeoff climb rates
        takeoff.get_climb_rates(target_climb_rate, takeoff_climb_rate);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->add_takeoff_climb_rate(takeoff_climb_rate, G_Dt);
        pos_control->update_z_controller();
        break;

    case AltHold_Landed:
        // set motors to spin-when-armed if throttle below deadzone, otherwise full range (but motors will only spin at min throttle)
        if (target_climb_rate < 0.0f) {
            motors->set_desired_spool_state(AP_Motors::DESIRED_SPIN_WHEN_ARMED);
        } else {
            motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
        }

#if FRAME_CONFIG == HELI_FRAME    
        if (heli_flags.init_targets_on_arming) {
            attitude_control->reset_rate_controller_I_terms();
            attitude_control->set_yaw_target_to_current_heading();
            if (motors->get_interlock()) {
                heli_flags.init_targets_on_arming=false;
            }
        }
#else
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->set_yaw_target_to_current_heading();
#endif
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        pos_control->relax_alt_hold_controllers(0.0f);   // forces throttle output to go to zero
        pos_control->update_z_controller();
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

        // adjust climb rate using rangefinder
        target_climb_rate = get_surface_tracking_climb_rate(target_climb_rate, pos_control->get_alt_target(), G_Dt);

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // call position controller
        pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
        pos_control->update_z_controller();
        break;
    }
}
