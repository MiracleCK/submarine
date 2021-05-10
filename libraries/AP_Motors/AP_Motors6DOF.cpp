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
#include "AP_Motors6DOF.h"
#include "AP_Logger/AP_Logger.h"

extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_Motors6DOF::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),
    // @Param: 1_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("1_DIRECTION", 1, AP_Motors6DOF, _motor_reverse[0], 1),

    // @Param: 2_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("2_DIRECTION", 2, AP_Motors6DOF, _motor_reverse[1], 1),

    // @Param: 3_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("3_DIRECTION", 3, AP_Motors6DOF, _motor_reverse[2], 1),

    // @Param: 4_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("4_DIRECTION", 4, AP_Motors6DOF, _motor_reverse[3], 1),

    // @Param: 5_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("5_DIRECTION", 5, AP_Motors6DOF, _motor_reverse[4], 1),

    // @Param: 6_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("6_DIRECTION", 6, AP_Motors6DOF, _motor_reverse[5], 1),

    // @Param: 7_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("7_DIRECTION", 7, AP_Motors6DOF, _motor_reverse[6], 1),

    // @Param: 8_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("8_DIRECTION", 8, AP_Motors6DOF, _motor_reverse[7], 1),

    // @Param: FV_CPLNG_K
    // @DisplayName: Forward/vertical to pitch decoupling factor
    // @Description: Used to decouple pitch from forward/vertical motion. 0 to disable, 1.2 normal
    // @Range: 0.0 1.5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FV_CPLNG_K", 9, AP_Motors6DOF, _forwardVerticalCouplingFactor, 1.0),

    // @Param: 9_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("9_DIRECTION", 10, AP_Motors6DOF, _motor_reverse[8], 1),

    // @Param: 10_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("10_DIRECTION", 11, AP_Motors6DOF, _motor_reverse[9], 1),

    // @Param: 11_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("11_DIRECTION", 12, AP_Motors6DOF, _motor_reverse[10], 1),

    // @Param: 12_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("12_DIRECTION", 13, AP_Motors6DOF, _motor_reverse[11], 1),

    // @Param: 1_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("1_MAPPING", 14, AP_Motors6DOF, _motor_mapping[0], 1),

    // @Param: 2_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("2_MAPPING", 15, AP_Motors6DOF, _motor_mapping[1], 2),

    // @Param: 3_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("3_MAPPING", 16, AP_Motors6DOF, _motor_mapping[2], 3),

    // @Param: 4_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("4_MAPPING", 17, AP_Motors6DOF, _motor_mapping[3], 4),

    // @Param: 5_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("5_MAPPING", 18, AP_Motors6DOF, _motor_mapping[4], 5),

    // @Param: 6_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("6_MAPPING", 19, AP_Motors6DOF, _motor_mapping[5], 6),

    // @Param: 7_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("7_MAPPING", 20, AP_Motors6DOF, _motor_mapping[6], 7),

    // @Param: 8_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("8_MAPPING", 21, AP_Motors6DOF, _motor_mapping[7], 8),

	// @Param: 1_DEADZONE
    // @DisplayName: Motor dead zone
    // @Description: Used to correct motor dead zone
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("1_DEADZONE", 22, AP_Motors6DOF, _motor_deadzone[0], 0),

    // @Param: 2_DEADZONE
    // @DisplayName: Motor dead zone
    // @Description: Used to correct motor dead zone
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("2_DEADZONE", 23, AP_Motors6DOF, _motor_deadzone[1], 0),

    // @Param: 3_DEADZONE
    // @DisplayName: Motor dead zone
    // @Description: Used to correct motor dead zone
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("3_DEADZONE", 24, AP_Motors6DOF, _motor_deadzone[2], 0),

    // @Param: 4_DEADZONE
    // @DisplayName: Motor dead zone
    // @Description: Used to correct motor dead zone
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("4_DEADZONE", 25, AP_Motors6DOF, _motor_deadzone[3], 0),

    // @Param: 5_DEADZONE
    // @DisplayName: Motor dead zone
    // @Description: Used to correct motor dead zone
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("5_DEADZONE", 26, AP_Motors6DOF, _motor_deadzone[4], 0),

    // @Param: 6_DEADZONE
    // @DisplayName: Motor dead zone
    // @Description: Used to correct motor dead zone
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("6_DEADZONE", 27, AP_Motors6DOF, _motor_deadzone[5], 0),

    // @Param: 7_DEADZONE
    // @DisplayName: Motor dead zone
    // @Description: Used to correct motor dead zone
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("7_DEADZONE", 28, AP_Motors6DOF, _motor_deadzone[6], 0),

    // @Param: 8_DEADZONE
    // @DisplayName: Motor dead zone
    // @Description: Used to correct motor dead zone
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("8_DEADZONE", 29, AP_Motors6DOF, _motor_deadzone[7], 0),
    
    AP_GROUPEND
};

void AP_Motors6DOF::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    // hard coded config for supported frames
    switch ((sub_frame_t)frame_class) {
        //                 Motor #              Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor  Testing Order
    case SUB_FRAME_BLUEROV1:
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     -0.5f,          0.5f,           0,              0.45f,              0,                  0,              3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0.5f,           0.5f,           0,              0.45f,              0,                  0,              4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     0,              -1.0f,          0,              1.0f,               0,                  0,              5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -0.25f,         0,              0,              0,                  0,                  1.0f,           6);
        break;

    case SUB_FRAME_VECTORED_6DOF_90DEG:
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     1.0f,           1.0f,           0,              1.0f,               0,                  0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     1.0f,           -1.0f,          0,              1.0f,               0,                  0,              3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0,              0,              0,              0,                  0,                  1.0f,           4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     0,              0,              0,              0,                  0,                  1.0f,           5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          1.0f,           0,              1.0f,               0,                  0,              6);
        add_motor_raw_6dof(AP_MOTORS_MOT_7,     0,              0,              -1.0f,          0,                  1.0f,               0,              7);
        add_motor_raw_6dof(AP_MOTORS_MOT_8,     -1.0f,          -1.0f,          0,              1.0f,               0,                  0,              8);
        break;

    case SUB_FRAME_VECTORED_6DOF:
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              1.0f,           0,                  -1.0f,              1.0f,           1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              -1.0f,          0,                  -1.0f,              -1.0f,          2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     0,              0,              -1.0f,          0,                  1.0f,               1.0f,           3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0,              0,              1.0f,           0,                  1.0f,               -1.0f,          4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     1.0f,           -1.0f,          0,              -1.0f,              0,                  0,              5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          -1.0f,          0,              -1.0f,              0,                  0,              6);
        add_motor_raw_6dof(AP_MOTORS_MOT_7,     1.0f,           1.0f,           0,              -1.0f,              0,                  0,              7);
        add_motor_raw_6dof(AP_MOTORS_MOT_8,     -1.0f,          1.0f,           0,              -1.0f,              0,                  0,              8);
        break;

    case SUB_FRAME_VECTORED:
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              1.0f,           0,                  -1.0f,              1.0f,           1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              -1.0f,          0,                  -1.0f,              -1.0f,          2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     0,              0,              -1.0f,          0,                  1.0f,               1.0f,           3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     0,              0,              1.0f,           0,                  1.0f,               -1.0f,          4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     1.0f,           0,              0,              -1.0f,              0,                  0,              5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          0,              0,              -1.0f,              0,                  0,              6);
        break;

    case SUB_FRAME_CUSTOM:
        // Put your custom motor setup here
        #ifdef CUSTOM_MOTORS_MOT_1
            add_motor_raw_6dof(AP_MOTORS_MOT_1, CUSTOM_MOTORS_MOT_1, 1);
        #endif
        #ifdef CUSTOM_MOTORS_MOT_2
            add_motor_raw_6dof(AP_MOTORS_MOT_2, CUSTOM_MOTORS_MOT_2, 2);
        #endif
        #ifdef CUSTOM_MOTORS_MOT_3
            add_motor_raw_6dof(AP_MOTORS_MOT_3, CUSTOM_MOTORS_MOT_3, 3);
        #endif
        #ifdef CUSTOM_MOTORS_MOT_4
            add_motor_raw_6dof(AP_MOTORS_MOT_4, CUSTOM_MOTORS_MOT_4, 4);
        #endif
        #ifdef CUSTOM_MOTORS_MOT_5
            add_motor_raw_6dof(AP_MOTORS_MOT_5, CUSTOM_MOTORS_MOT_5, 5);
        #endif
        #ifdef CUSTOM_MOTORS_MOT_6
            add_motor_raw_6dof(AP_MOTORS_MOT_6, CUSTOM_MOTORS_MOT_6, 6);
        #endif
        #ifdef CUSTOM_MOTORS_MOT_7
            add_motor_raw_6dof(AP_MOTORS_MOT_7, CUSTOM_MOTORS_MOT_7, 7);
        #endif
        #ifdef CUSTOM_MOTORS_MOT_8
            add_motor_raw_6dof(AP_MOTORS_MOT_8, CUSTOM_MOTORS_MOT_8, 8);
        #endif
        break;

    case SUB_FRAME_SIMPLEROV_3:
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     0,              0,              0,              -1.0f,              0,                  0,              3);
        break;
    case SUB_FRAME_SIMPLEROV_4:
    case SUB_FRAME_SIMPLEROV_5:
    default:
        add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,     1.0f,           0,              0,              -1.0f,              0,                  0,              3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,     -1.0f,          0,              0,              -1.0f,              0,                  0,              4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     0,              0,              0,              0,                  0,                  1.0f,           5);
        break;
    }
}

void AP_Motors6DOF::add_motor_raw_6dof(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float throttle_fac, float forward_fac, float lat_fac, uint8_t testing_order)
{
    //Parent takes care of enabling output and setting up masks
    add_motor_raw(motor_num, roll_fac, pitch_fac, yaw_fac, testing_order);

    //These are additional parameters for an ROV
    _throttle_factor[motor_num] = throttle_fac;
    _forward_factor[motor_num] = forward_fac;
    _lateral_factor[motor_num] = lat_fac;
}

// output_min - sends minimum values out to the motors
void AP_Motors6DOF::output_min()
{
    int8_t i;

    // set limits flags
    limit.roll = true;
    limit.pitch = true;
    limit.yaw = true;
    limit.throttle_lower = false;
    limit.throttle_upper = false;

    // fill the motor_out[] array for HIL use and send minimum value to each motor
    // ToDo find a field to store the minimum pwm instead of hard coding 1500
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, 0);
        }
    }
}

int16_t AP_Motors6DOF::calc_thrust_to_pwm(float thrust_in) const
{
    return constrain_int16(thrust_in * 2200, _throttle_radio_min, _throttle_radio_max);
}

void AP_Motors6DOF::output_to_motors()
{
    int8_t i;
    int16_t motor_out[AP_MOTORS_MAX_NUM_MOTORS];    // final pwm values sent to the motor

    switch (_spool_state) {
    case SpoolState::SHUT_DOWN:
        // sends minimum values out to the motors
        // set motor output based on thrust requests
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = 0;
            }
        }
        break;
    case SpoolState::GROUND_IDLE:
        // sends output to motors when armed but not flying
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = 0;
            }
        }
        break;
    case SpoolState::SPOOLING_UP:
    case SpoolState::THROTTLE_UNLIMITED:
    case SpoolState::SPOOLING_DOWN:
        // set motor output based on thrust requests
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                motor_out[i] = calc_thrust_to_pwm(_thrust_rpyt_out[i]);
            }
        }
        break;
    }

    // send output to each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
        	if(_motor_deadzone[i] > 0) {
        		int16_t min, max;

        		min = 0 - _motor_deadzone[i];
        		max = 0 + _motor_deadzone[i];
	        	if(motor_out[i] > min && motor_out[i] < 0)
	        		motor_out[i] = min;
	        	if(motor_out[i] > 0 && motor_out[i] < max)
	        		motor_out[i] = max;
        	}
        	
			if(i==0) {
	        	if(motor_out[i] >= 0) {
	        		palWriteLine(HAL_GPIO_PIN_M1_CTRL, 1);
	        	} else {
	        		palWriteLine(HAL_GPIO_PIN_M1_CTRL, 0);
	        	}
        	}

        	if(i==1) {
	        	if(motor_out[i] >= 0) {
	        		palWriteLine(HAL_GPIO_PIN_M2_CTRL, 1);
	        	} else {
	        		palWriteLine(HAL_GPIO_PIN_M2_CTRL, 0);
	        	}
        	}

        	//hal.shell->printf("%d:%d\r\n", _motor_mapping[i] - 1, motor_out[i]);
            rc_write(_motor_mapping[i] - 1, abs(motor_out[i]));

			if(0) {
		        static uint32_t _startup_ms = 0;

		        if(_startup_ms == 0) {
					_startup_ms = AP_HAL::millis();
		        }

		        if(AP_HAL::millis() - _startup_ms > 1000) {
					_startup_ms = AP_HAL::millis();
					
				    hal.shell->printf("_throttle_radio_min %d,_throttle_radio_max %d\r\n", _throttle_radio_min, _throttle_radio_max);
				}
			}
        }
    }
}

float AP_Motors6DOF::get_current_limit_max_throttle()
{
    return 1.0f;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/lateral commands
extern bool is_param_print(void);
extern bool is_dbg_motor;
void AP_Motors6DOF::output_armed_stabilizing()
{
    if ((sub_frame_t)_last_frame_class == SUB_FRAME_VECTORED) {
        output_armed_stabilizing_vectored();
    } else if ((sub_frame_t)_last_frame_class == SUB_FRAME_VECTORED_6DOF) {
        output_armed_stabilizing_vectored_6dof();
    } else {
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

		if(0) {
	        static uint32_t _startup_ms = 0;

	        if(_startup_ms == 0) {
				_startup_ms = AP_HAL::millis();
	        }

	        if(AP_HAL::millis() - _startup_ms > 100) {
				_startup_ms = AP_HAL::millis();
				
			    //printf("roll %f\r\n", _roll_in);
			    //printf("pitch %f\r\n", _pitch_in);
			    printf("yaw %.04f\r\n", _yaw_in);
			    //printf("throttle %f\r\n", _throttle_in);
			    printf("forward %.04f\r\n", _forward_in);
			    //printf("lateral %f\r\n", _lateral_in);
			    //printf("\r\n");

			    AP::logger().Write("MDOF", "TimeUS,RT,PT,YT,FT,LT,TT", "Qffffff", 
	                            AP_HAL::micros64(),
	                            (double)roll_thrust, 
	                            (double)pitch_thrust,
	                            (double)yaw_thrust, 
	                            (double)forward_thrust,
	                            (double)lateral_thrust,
	                            (double)throttle_thrust);
		    }
		}

#if 0
        forward_thrust = forward_thrust * cosf(_pitch_thr) + throttle_thrust * sinf(_pitch_thr);
        lateral_thrust = lateral_thrust * cosf(_roll_thr) + throttle_thrust * sinf(_roll_thr);
        throttle_thrust = throttle_thrust * cosf(_pitch_thr) * cosf(_roll_thr) 
                        - _forward_in * sinf(_pitch_thr) 
                        + _lateral_in * sinf(_roll_thr);
#endif
        if (is_param_print() && is_dbg_motor) {
            printf("pitch = %2.2f roll = %2.2f\r\n", _pitch_thr, _roll_thr);
            printf("forward = %1.3f lateral = %1.3f throttle = %1.3f\r\n", forward_thrust, lateral_thrust, throttle_thrust);
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

        if (is_param_print() && is_dbg_motor) {
            printf("ryp_out:\r\n");
        }

        // calculate roll, pitch and yaw for each motor
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                rpy_out[i] = roll_thrust * _roll_factor[i] +
                             pitch_thrust * _pitch_factor[i] +
                             yaw_thrust * _yaw_factor[i];
                if (is_param_print() && is_dbg_motor) {
                    printf("%2.2f ", rpy_out[i]);
                }
            }
        }

        // calculate linear command for each motor
        // linear factors should be 0.0 or 1.0 for now
        if (is_param_print() && is_dbg_motor) {
            printf("\r\n");
            printf("linear_out:\r\n");
        }
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                linear_out[i] = throttle_thrust * _throttle_factor[i] +
                                forward_thrust * _forward_factor[i] +
                                lateral_thrust * _lateral_factor[i];
                if (is_param_print() && is_dbg_motor) {
                    printf("%2.2f ", linear_out[i]);
                }
            }
        }

        // Calculate final output for each motor
        if (is_param_print() && is_dbg_motor) {
            printf("\r\n");
            printf("_thrust_rpyt_out:\r\n");
        }
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                _thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpy_out[i] + linear_out[i]),-1.0f,1.0f);
                if (is_param_print() && is_dbg_motor) {
                    printf("%2.2f ", _thrust_rpyt_out[i]);
                }
            }
        }

        if (is_param_print() && is_dbg_motor) {
            printf("\r\n\r\n");
        }
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

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] *= _output_limited;
        }
    }
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/lateral commands
void AP_Motors6DOF::output_armed_stabilizing_vectored()
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

    float rpy_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    float linear_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor

    // initialize limits flags
    limit.roll= false;
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

    // calculate roll, pitch and yaw for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rpy_out[i] = roll_thrust * _roll_factor[i] +
                         pitch_thrust * _pitch_factor[i] +
                         yaw_thrust * _yaw_factor[i];
        }
    }

    float forward_coupling_limit = 1-_forwardVerticalCouplingFactor*float(fabsf(throttle_thrust));
    if (forward_coupling_limit < 0) {
        forward_coupling_limit = 0;
    }
    int8_t forward_coupling_direction[] = {-1,-1,1,1,0,0,0,0,0,0,0,0};

    // calculate linear command for each motor
    // linear factors should be 0.0 or 1.0 for now
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {

            float forward_thrust_limited = forward_thrust;

            // The following statements decouple forward/vertical hydrodynamic coupling on
            // vectored ROVs. This is done by limiting the maximum output of the "rear" vectored
            // thruster (where "rear" depends on direction of travel).
            if (!is_zero(forward_thrust_limited)) {
                if ((forward_thrust < 0) == (forward_coupling_direction[i] < 0) && forward_coupling_direction[i] != 0) {
                    forward_thrust_limited = constrain_float(forward_thrust, -forward_coupling_limit, forward_coupling_limit);
                }
            }

            linear_out[i] = throttle_thrust * _throttle_factor[i] +
                            forward_thrust_limited * _forward_factor[i] +
                            lateral_thrust * _lateral_factor[i];
        }
    }

    // Calculate final output for each motor
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpy_out[i] + linear_out[i]), -1.0f, 1.0f);
        }
    }
}

// Band Aid fix for motor normalization issues.
// TODO: find a global solution for managing saturation that works for all vehicles
void AP_Motors6DOF::output_armed_stabilizing_vectored_6dof()
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

    float rpt_out[AP_MOTORS_MAX_NUM_MOTORS]; // buffer so we don't have to multiply coefficients multiple times.
    float yfl_out[AP_MOTORS_MAX_NUM_MOTORS]; // 3 linear DOF mix for each motor
    float rpt_max;
    float yfl_max;

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

    // calculate roll, pitch and Throttle for each motor (only used by vertical thrusters)
    rpt_max = 1; //Initialized to 1 so that normalization will only occur if value is saturated
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rpt_out[i] = roll_thrust * _roll_factor[i] +
                         pitch_thrust * _pitch_factor[i] +
                         throttle_thrust * _throttle_factor[i];
            if (fabsf(rpt_out[i]) > rpt_max) {
                rpt_max = fabsf(rpt_out[i]);
            }
        }
    }

    // calculate linear/yaw command for each motor (only used for translational thrusters)
    // linear factors should be 0.0 or 1.0 for now
    yfl_max = 1; //Initialized to 1 so that normalization will only occur if value is saturated
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            yfl_out[i] = yaw_thrust * _yaw_factor[i] +
                         forward_thrust * _forward_factor[i] +
                         lateral_thrust * _lateral_factor[i];
            if (fabsf(yfl_out[i]) > yfl_max) {
                yfl_max = fabsf(yfl_out[i]);
            }
        }
    }

    // Calculate final output for each motor and normalize if necessary
    for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = constrain_float(_motor_reverse[i]*(rpt_out[i]/rpt_max + yfl_out[i]/yfl_max),-1.0f,1.0f);
        }
    }
}

Vector3f AP_Motors6DOF::get_motor_angular_factors(int motor_number) {
     if (motor_number < 0 || motor_number >= AP_MOTORS_MAX_NUM_MOTORS) {
        return Vector3f(0,0,0);
    }
    return Vector3f(_roll_factor[motor_number], _pitch_factor[motor_number], _yaw_factor[motor_number]);
}

bool AP_Motors6DOF::motor_is_enabled(int motor_number) {
    if (motor_number < 0 || motor_number >= AP_MOTORS_MAX_NUM_MOTORS) {
        return false;
    }
    return motor_enabled[motor_number];
}

bool AP_Motors6DOF::set_reversed(int motor_number, bool reversed) {
    if (motor_number < 0 || motor_number >= AP_MOTORS_MAX_NUM_MOTORS) {
        return false;
    }
    if (reversed) {
        _motor_reverse[motor_number].set_and_save(-1);
    } else {
        _motor_reverse[motor_number].set_and_save(1);
    }
    return true;
}
