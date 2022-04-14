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
 *       AP_Motors6DOF.cpp - ArduSub motors library
 */

#include <math.h>
#include <stdio.h>

#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>

#include "AP_Motors6DOF.h"

extern const AP_HAL::HAL& hal;

static void to_designer_order_out(float out[]);
static void motor_vector_force_debug(float rpy_out[], float linear_out[], float rpyt_out[], float ratio);

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/lateral commands
extern bool is_param_print(void);
extern bool is_dbg_motor;
extern float correct_pitch_thr;
extern float correct_roll_thr;
void AP_Motors6DOF::output_armed_stabilizing_custom()
{
    uint8_t i;                          // general purpose counter
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, +/- 1.0
    float   forward_thrust;             // forward thrust input value, +/- 1.0
    float   lateral_thrust;             // lateral thrust input value, +/- 1.0

    roll_thrust = (_roll_in + _roll_in_ff);
    pitch_thrust = (_pitch_in + _pitch_in_ff);
    yaw_thrust = (_yaw_in + _yaw_in_ff);
    throttle_thrust = get_throttle_bidirectional();
    forward_thrust = _forward_in;
    lateral_thrust = _lateral_in;
    // printf("pitch_thrust: _pitch_in %2.4f _pitch_in_ff %2.4f\r\n", _pitch_in, _pitch_in_ff);

    // test real data
    // roll_thrust = 0.04;
    // pitch_thrust = 0.594;
    // yaw_thrust = -0.001;
    // throttle_thrust = 0.284;
    // corrected_pitch = -0.02;
    // corrected_roll = 0;

    if (is_param_print() && is_dbg_motor) {
        printf("============================\r\n");
        printf("thrust: roll %2.4f pitch %2.4f yaw %2.4f\r\n", roll_thrust, pitch_thrust, yaw_thrust);
        printf("thurst: forward = %2.4f lateral = %2.4f throttle = %2.4f\r\n", forward_thrust, lateral_thrust, throttle_thrust);
    }

    Vector3f thrusts(forward_thrust, lateral_thrust, throttle_thrust);
    Vector3f euler_rad(0.0f, 0.0f, 0.0f);

    if (_thrust_decomposition_callback) {
        Vector3f thrusts_decomped = _thrust_decomposition_callback(euler_rad, thrusts, _throttle_in_bf);

        forward_thrust = thrusts_decomped.x;
        lateral_thrust = thrusts_decomped.y;
        throttle_thrust = thrusts_decomped.z;

        if (is_param_print() && is_dbg_motor) {
            printf("thrust decomposition: degree roll %4.2f pitch %4.2f \r\n", ToDeg(euler_rad.x), ToDeg(euler_rad.y));
            printf("thrust decomposition: forward %2.4f lateral %2.4f throttle %2.4f\r\n", forward_thrust, lateral_thrust, throttle_thrust);
        }
    } else {
        throttle_thrust = _throttle_in_bf;
    }

    if(1) {
		static uint32_t _startup_ms = 0;

		if(_startup_ms == 0) {
			_startup_ms = AP_HAL::millis();
		}

		if(AP_HAL::millis() - _startup_ms > 1000) {
			_startup_ms = AP_HAL::millis();
            hal.console->printf("******before********limited throttle output*************\n");
			hal.console->printf("roll_thrust %.02f\n", roll_thrust);
			hal.console->printf("pitch_thrust %.02f\n", pitch_thrust);
			hal.console->printf("yaw_thrust %.02f\n", yaw_thrust);
			hal.console->printf("throttle_thrust %.02f\n", throttle_thrust);
			hal.console->printf("forward_thrust %.02f\n", forward_thrust);
			hal.console->printf("lateral_thrust %.02f\n", lateral_thrust);
		}
	}

    if (motor_log_start) {
        AP::logger().Write("MDOF", "TimeUS,R,P,Y,RT,PT,YT,FTD,FT,LTD,LT,TTD,TT,BTT", "Qfffffffffffff", 
                            AP_HAL::micros64(),
                            (double)ToDeg(euler_rad.x),
                            (double)ToDeg(euler_rad.y),
                            (double)ToDeg(euler_rad.z),
                            (double)roll_thrust, 
                            (double)pitch_thrust,
                            (double)yaw_thrust, 
                            (double)thrusts.x,
                            (double)forward_thrust,
                            (double)thrusts.y,
                            (double)lateral_thrust,
                            (double)thrusts.z,
                            (double)throttle_thrust,
                            (double)_throttle_in_bf);

        motor_log_start = false;
    }

    float rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    float linear_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor

    // initialize limits flags
    limit.roll = false;
    limit.pitch = false;
    limit.yaw = false;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= -_throttle_thrust_max) {
        throttle_thrust = -_throttle_thrust_max;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // sanity check pitch is above zero and below current limited throttle
    if (roll_thrust <= -1) {
        roll_thrust = -1;
        limit.roll = true;
    }
    if (roll_thrust >= 1) {
        roll_thrust = 1;
        limit.roll = true;
    }

    // sanity check pitch is above zero and below current limited throttle
    if (pitch_thrust <= -1) {
        pitch_thrust = -1;
        limit.pitch = true;
    }
    if (pitch_thrust >= 1) {
        pitch_thrust = 1;
        // limit.pitch = true;
    }

    if(1) {
		static uint32_t _startup_ms = 0;

		if(_startup_ms == 0) {
			_startup_ms = AP_HAL::millis();
		}

		if(AP_HAL::millis() - _startup_ms > 1000) {
			_startup_ms = AP_HAL::millis();
            hal.console->printf("*****************limited throttle output*************\n");
			hal.console->printf("roll_thrust %.02f\n", roll_thrust);
			hal.console->printf("pitch_thrust %.02f\n", pitch_thrust);
			hal.console->printf("yaw_thrust %.02f\n", yaw_thrust);
			hal.console->printf("throttle_thrust %.02f\n", throttle_thrust);
			hal.console->printf("forward_thrust %.02f\n", forward_thrust);
			hal.console->printf("lateral_thrust %.02f\n", lateral_thrust);
		}
	}

    // calculate roll, pitch and yaw for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rpy_out[i] = roll_thrust * _roll_factor[i] +
                            pitch_thrust * _pitch_factor[i] +
                            yaw_thrust * _yaw_factor[i];
        }
    }

    // calculate linear command for each motor
    // linear factors should be 0.0 or 1.0 for now
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            linear_out[i] = throttle_thrust * _throttle_factor[i] +
                            forward_thrust * _forward_factor[i] +
                            lateral_thrust * _lateral_factor[i];
        }
    }

    // Calculate final output for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpy_out[i] + linear_out[i]),-1.0f,1.0f);
        }
    }

	if(0) {
		static uint32_t _startup_ms = 0;

		if(_startup_ms == 0) {
			_startup_ms = AP_HAL::millis();
		}

		if(AP_HAL::millis() - _startup_ms > 1000) {
			_startup_ms = AP_HAL::millis();

			for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
		        if (motor_enabled[i]) {
		            hal.console->printf("_thrust_rpyt_out %d: %.02f\n", i, _thrust_rpyt_out[i]);
		        }
		    }
		}
	}
	
    if (is_param_print() && is_dbg_motor) {
        float origin_rpyt_out[8];
        for (i = 0; i < 8; i++) {
            origin_rpyt_out[i] = _thrust_rpyt_out[i];
        }

        to_designer_order_out(origin_rpyt_out);
        
        printf("\r\nprintf as designer MOT_n order\r\n");
        printf("origin rpyt out:\r\n");
        for (i = 0; i < 8; i++) {
            printf("%2.4f ", origin_rpyt_out[i]);
        }
    }

    // correct nagative thurst with param ratio
    // M2 programed MOT_n      1   2   3   4   5   6   7   8
    //             propeller   R   L   R   L   L   R   R   L
    // int motor_lr_factor[8] = {-1,  1, -1,  1,  1, -1, -1,  1}; // L/R propeller factor, to make positive thrust with positive sign
    // float thrust_rpyt_out_max = 1;
    
    if (is_param_print() && is_dbg_motor) {
        motor_vector_force_debug(rpy_out, linear_out, _thrust_rpyt_out, 0.0f);
    }

    const AP_BattMonitor &battery = AP::battery();

	// Current limiting
    float _batt_current;
    if (_batt_current_max <= 0.0f || !battery.current_amps(_batt_current)) {
        if (is_param_print() && is_dbg_motor) {
            printf("not used battery motor output limit\r\n");
            printf("\r\n\r\n");
        }
        return;
    }

    if (is_param_print() && is_dbg_motor) {
        printf("do battery motor output limit\r\n");
        printf("\r\n\r\n");
    }

    float _batt_current_delta = _batt_current - _batt_current_last;

    float loop_interval = 1.0f/_loop_rate;

    float _current_change_rate = _batt_current_delta / loop_interval;

    float predicted_current = _batt_current + (_current_change_rate * loop_interval * 5);

    float batt_current_ratio = _batt_current/_batt_current_max;

    float predicted_current_ratio = predicted_current/_batt_current_max;
    _batt_current_last = _batt_current;

    if (predicted_current > _batt_current_max * 1.5f) {
        batt_current_ratio = 2.5f;
    } else if (_batt_current < _batt_current_max && predicted_current > _batt_current_max) {
        batt_current_ratio = predicted_current_ratio;
    }
    _output_limited += (loop_interval/(loop_interval+_batt_current_time_constant)) * (1 - batt_current_ratio);

    _output_limited = constrain_float(_output_limited, 0.0f, 1.0f);

    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] *= _output_limited;
        }
    }
}

void to_designer_order_out(float out[]) {
    // map programed MOT_1/2/... to Structural designer defined MOT_1/2/...
    // MOT1:4 MOT2:3 MOT3:2 MOT4:1 MOT5:6 MOT6:7 MOT7:5 MOT8:8
    int motor_stru_mapping[8] = {4, 3, 2, 1, 6, 7, 5, 8};

    int designer_motor_order;
    int i;
    float tmp[8];

    for (i = 0; i < 8; i++) {
        designer_motor_order = motor_stru_mapping[i];
        tmp[designer_motor_order - 1] = out[i];
    }

    for (i = 0; i < 8; i++) {
        out[i] = tmp[i];
    }
}

void motor_vector_force_debug(float rpy_out[], float linear_out[], float rpyt_out[], float ratio) {
    // test data from designer table
    // our thrust -1 means clockwise, it revesed to design
    // so do reverse when define
    // uncomment to do test
    // the output
    // F_X F_Y F_Z T_FX T_FY T_FZ should be
    // 0.306151821 9.615554152 12.62041482 -941.0516601 19.54821892 61.54757141
    // float motor_thrust_test[8] = {-0.12, 0.2, -0.24, 0.31, 0.3, 0.64, -0.19, -0.54};
    // rpyt_out = motor_thrust_test;

    // programed MOT_n         1   2   3   4   5   6   7   8
    //             propeller   R   L   R   L   L   R   R   L
    int motor_lr_factor[8] = {-1,  1, -1,  1,  1, -1, -1,  1}; // L/R propeller factor, to make positive thrust with positive sign
    
    // designer order
    float k_tn[8];

    int i;

    float designer_rpy_out[8], designer_linear_out[8], designer_rpyt_out[8];

    for (i = 0; i < 8; i++) {
        designer_rpy_out[i] = rpy_out[i];
        designer_linear_out[i] = linear_out[i];
        designer_rpyt_out[i] = rpyt_out[i]; 

        k_tn[i] = rpyt_out[i] * motor_lr_factor[i];
    }

    to_designer_order_out(designer_rpy_out);
    to_designer_order_out(designer_linear_out);
    to_designer_order_out(designer_rpyt_out);
    to_designer_order_out(k_tn);

    for (i = 0; i < 8; i++) {
        if (k_tn[i] < 0 && !is_zero(ratio)) {
            k_tn[i] *= ratio;
        }
    }

    printf("\r\nprintf as designer MOT_n order");
    printf("\r\nrpy out:\r\n");
    for (i = 0; i < 8; i++) {
        printf("%2.4f ", designer_rpy_out[i]);
    }
    printf("\r\nlinear out:\r\n");
    for (i = 0; i < 8; i++) {
        printf("%2.4f ", designer_linear_out[i]);
    }
    printf("\r\nrpyt out:\r\n");
    for (i = 0; i < 8; i++) {
        printf("%2.4f ", designer_rpyt_out[i]);
    }
    printf("\r\nKtn:\r\n");
    for (i = 0; i < 8; i++) {
        printf("%2.4f ", k_tn[i]);
    }

    float f_max = 13.5f;
    // designer order
    float cos_alpha[AP_MOTORS_MAX_NUM_MOTORS] = {-0.377965211, 0.377965211, 0.377965211, -0.377965211,
        -0.377965211, 0.377965211, 0.377965211, -0.377965211};
    float cos_beta[AP_MOTORS_MAX_NUM_MOTORS] = {0.654653741, 0.654653741, 0.654653741, 0.654653741,
        -0.654653741, -0.654653741, -0.654653741, -0.654653741};
    float cos_gamma[AP_MOTORS_MAX_NUM_MOTORS] = {0.654653741, 0.654653741, -0.654653741, -0.654653741,
        0.654653741, 0.654653741, -0.654653741, -0.654653741};
    float x_tn[AP_MOTORS_MAX_NUM_MOTORS] = {87.5, -87.5, -87.5, 87.5, 87.5, -87.5, -87.5, 87.5};
    float y_tn[AP_MOTORS_MAX_NUM_MOTORS] = {150, 150, 150, 150, -150, -150, -150, -150};
    float z_tn[AP_MOTORS_MAX_NUM_MOTORS] = {40, 40, -40, -40, 40, 40, -40, -40};
    float f_x[AP_MOTORS_MAX_NUM_MOTORS], 
            f_y[AP_MOTORS_MAX_NUM_MOTORS], 
            f_z[AP_MOTORS_MAX_NUM_MOTORS], 
            t_fx[AP_MOTORS_MAX_NUM_MOTORS], 
            t_fy[AP_MOTORS_MAX_NUM_MOTORS], 
            t_fz[AP_MOTORS_MAX_NUM_MOTORS];
    float f_x_sum = 0.0f, f_y_sum = 0.0f, f_z_sum = 0.0f, 
            t_fx_sum = 0.0f, t_fy_sum = 0.0f, t_fz_sum = 0.0f;
    for (i = 0; i < 8; i++) {
        f_x[i] = f_max * k_tn[i] * cos_alpha[i];
        f_y[i] = f_max * k_tn[i] * cos_beta[i];
        f_z[i] = f_max * k_tn[i] * cos_gamma[i];
        t_fx[i] = y_tn[i] * f_z[i] - z_tn[i] * f_y[i];
        t_fy[i] = z_tn[i] * f_x[i] - x_tn[i] * f_z[i];
        t_fz[i] = x_tn[i] * f_y[i] - y_tn[i] * f_x[i];
    }
    for (i = 0; i < 8; i++) {
        f_x_sum += f_x[i];
        f_y_sum += f_y[i];
        f_z_sum += f_z[i];
        t_fx_sum += t_fx[i];
        t_fy_sum += t_fy[i];
        t_fz_sum += t_fz[i];
    }

    printf("\r\n F_X         F_Y         F_Z         T_FX        T_FY        T_FZ\r\n");
    printf("%6.4f %6.4f %6.4f %6.4f %6.4f  %6.4f\r\n", f_x_sum, f_y_sum, f_z_sum,
        t_fx_sum, t_fy_sum, t_fz_sum);

    printf("\r\n\r\n");
}
