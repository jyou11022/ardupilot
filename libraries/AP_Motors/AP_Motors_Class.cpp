/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Motors.cpp - ArduCopter motors library
 *       Code by RandyMackay. DIYDrones.com
 *
 */

#include "AP_Motors_Class.h"
#include <AP_HAL/AP_HAL.h>
#include <SRV_Channel/SRV_Channel.h>
#include <GCS_MAVLink/GCS.h>
#include <DataFlash/DataFlash.h>

extern const AP_HAL::HAL& hal;

// singleton instance
AP_Motors *AP_Motors::_instance;

// Constructor
AP_Motors::AP_Motors(uint16_t loop_rate, uint16_t speed_hz) :
    _loop_rate(loop_rate),
    _speed_hz(speed_hz),
    _roll_in(0.0f),
    _pitch_in(0.0f),
    _yaw_in(0.0f),
    _throttle_in(0.0f),
    _throttle_avg_max(0.0f),
    _throttle_filter(),
    _spool_desired(DESIRED_SHUT_DOWN),
    _air_density_ratio(1.0f),
    _motor_fast_mask(0)
{
    _instance = this;
    
    // init other flags
    _flags.armed = false;
    _flags.interlock = false;
    _flags.initialised_ok = false;

    // setup throttle filtering
    _throttle_filter.set_cutoff_frequency(0.0f);
    _throttle_filter.reset(0.0f);

    // init limit flags
    limit.roll_pitch = true;
    limit.yaw = true;
    limit.throttle_lower = true;
    limit.throttle_upper = true;

    // naviator vars --> set defaults
    motor_min_enable = false;
    motor_min = 1055;
    for (int i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        motor_medium[i] = MOTORS_REGULAR; // default to reg
    }
};

void AP_Motors::armed(bool arm)
{
    if (_flags.armed != arm) {
        _flags.armed = arm;
        AP_Notify::flags.armed = arm;
        if (!arm) {
            save_params_on_disarm();
        }
    }
};

// pilot input in the -1 ~ +1 range for roll, pitch and yaw. 0~1 range for throttle
void AP_Motors::set_radio_passthrough(float roll_input, float pitch_input, float throttle_input, float yaw_input)
{
    _roll_radio_passthrough = roll_input;
    _pitch_radio_passthrough = pitch_input;
    _throttle_radio_passthrough = throttle_input;
    _yaw_radio_passthrough = yaw_input;
}

/*
  write to an output channel
 */
void AP_Motors::rc_write(uint8_t chan, uint16_t pwm)
{
    SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(chan);
    SRV_Channels::set_output_pwm(function, pwm);
}

/*
  write to an output channel for an angle actuator
 */
void AP_Motors::rc_write_angle(uint8_t chan, int16_t angle_cd)
{
    SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(chan);
    SRV_Channels::set_output_scaled(function, angle_cd);
}

/*
  set frequency of a set of channels
 */
void AP_Motors::rc_set_freq(uint32_t mask, uint16_t freq_hz)
{
    if (freq_hz > 50) {
        _motor_fast_mask |= mask;
    }

    mask = rc_map_mask(mask);
    hal.rcout->set_freq(mask, freq_hz);

    switch (pwm_type(_pwm_type.get())) {
    case PWM_TYPE_ONESHOT:
        if (freq_hz > 50 && mask != 0) {
            hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        }
        break;
    case PWM_TYPE_ONESHOT125:
        if (freq_hz > 50 && mask != 0) {
            hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_ONESHOT125);
        }
        break;
    case PWM_TYPE_BRUSHED:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        break;
    case PWM_TYPE_DSHOT150:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_DSHOT150);
        break;
    case PWM_TYPE_DSHOT300:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_DSHOT300);
        break;
    case PWM_TYPE_DSHOT600:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_DSHOT600);
        break;
    case PWM_TYPE_DSHOT1200:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_DSHOT1200);
        break;
    default:
        hal.rcout->set_output_mode(mask, AP_HAL::RCOutput::MODE_PWM_NORMAL);
        break;
    }
}

/*
  map an internal motor mask to real motor mask, accounting for
  SERVOn_FUNCTION mappings, and allowing for multiple outputs per
  motor number
 */
uint32_t AP_Motors::rc_map_mask(uint32_t mask) const
{
    uint32_t mask2 = 0;
    for (uint8_t i=0; i<32; i++) {
        uint32_t bit = 1UL<<i;
        if (mask & bit) {
            SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(i);
            mask2 |= SRV_Channels::get_output_channel_mask(function);
        }
    }
    return mask2;
}

/*
  add a motor, setting up default output function as needed
 */
void AP_Motors::add_motor_num(int8_t motor_num)
{
    // ensure valid motor number is provided
    if( motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS ) {
        uint8_t chan;
        SRV_Channel::Aux_servo_function_t function = SRV_Channels::get_motor_function(motor_num);
        SRV_Channels::set_aux_channel_default(function, motor_num);
        if (!SRV_Channels::find_channel(function, chan)) {
            gcs().send_text(MAV_SEVERITY_ERROR, "Motors: unable to setup motor %u", motor_num);
        }
    }
}

void AP_Motors::update_esc_state(int index, float voltage, float current, float temperature, int rpm, bool senses_water, int meas_kv){
    if (AP_HAL::millis() - esc_status_states[index].last_reading_ms > 3000){
        gcs().send_text(MAV_SEVERITY_INFO, "Found esc %d", index);
    }

    esc_status_states[index].voltage = voltage;
    esc_status_states[index].current = current;
    esc_status_states[index].temperature = temperature;
    esc_status_states[index].rpm = rpm;
    esc_status_states[index].is_underwater = senses_water;
    esc_status_states[index].meas_kv = meas_kv;
    esc_status_states[index].last_reading_ms = AP_HAL::millis();
}

void AP_Motors::Log_Write_ESCs(){
    uint32_t tnow = AP_HAL::millis();
    for(uint8_t i = 0; i<8; i++){
        if (tnow - esc_status_states[i].last_reading_ms < 3000){
            DataFlash_Class::instance()->Log_Write_ESC(
                i+1,
                esc_status_states[i].voltage,
                esc_status_states[i].current,
                esc_status_states[i].temperature,
                esc_status_states[i].rpm,
                esc_status_states[i].is_underwater,
                esc_status_states[i].meas_kv);
        }
    }
}

float AP_Motors::get_voltage(){
    float sum = 0;
    int count = 0;

    uint32_t tnow = AP_HAL::millis();

    for(int i=0; i<8; i++){
        if (esc_status_states[i].voltage > 3 && tnow-esc_status_states[i].last_reading_ms < 3000){
            sum += esc_status_states[i].voltage;
            count ++;
        } 
    }
    return sum / count;
}
float AP_Motors::get_current(){
    float sum = 0;

    uint32_t tnow = AP_HAL::millis();

    for(int i=0; i<8; i++){
        if (tnow - esc_status_states[i].last_reading_ms < 3000){
            sum += esc_status_states[i].current;
        }
    }

    return sum;
}

bool AP_Motors::control_state_water(){
    for(uint8_t i=0; i<4; i++){
        if(motor_medium[i] == MOTORS_AIR){
           return false;
        }
    }
    return true;
} 

bool AP_Motors::is_underwater(){
    return top_is_underwater() || bot_is_underwater();
}

bool AP_Motors::top_is_underwater(){
    uint32_t tnow = AP_HAL::millis();    

    for(int i=0; i<4; i++){
        if ((tnow - esc_status_states[i].last_reading_ms < 3000 && esc_status_states[i].is_underwater) || motor_medium[i] == MOTORS_WATER) return true;
    }
    return false;
}

bool AP_Motors::bot_is_underwater(){
    uint32_t tnow = AP_HAL::millis();

    for(int i=4; i<8; i++){
        if ((tnow - esc_status_states[i].last_reading_ms < 3000 && esc_status_states[i].is_underwater) || motor_medium[i] == MOTORS_WATER) return true;
    }
    return false;
}

uint8_t AP_Motors::ESC_unhealthy(){
    uint32_t tnow = AP_HAL::millis();
    //if any ESC hasn't responded in the past 3 seconds, it's probably dead
    for(uint8_t i=0; i<8; i++){
        if (tnow - esc_status_states[i].last_reading_ms > 3000){
            return i+1;
        }
    }
    //if passed all ESCs, default to true
    return 0;
}
