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
#include "AP_Motors6DOF_M2.h"

extern const AP_HAL::HAL& hal;

// parameters for the motor class
const AP_Param::GroupInfo AP_Motors6DOF_M2::var_info[] = {
    AP_NESTEDGROUPINFO(AP_MotorsMulticopter, 0),
    // @Param: 1_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("1_DIRECTION", 1, AP_Motors6DOF_M2, _motor_reverse[0], 1),

    // @Param: 2_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("2_DIRECTION", 2, AP_Motors6DOF_M2, _motor_reverse[1], 1),

    // @Param: 3_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("3_DIRECTION", 3, AP_Motors6DOF_M2, _motor_reverse[2], 1),

    // @Param: 4_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("4_DIRECTION", 4, AP_Motors6DOF_M2, _motor_reverse[3], 1),

    // @Param: 5_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("5_DIRECTION", 5, AP_Motors6DOF_M2, _motor_reverse[4], 1),

    // @Param: 6_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("6_DIRECTION", 6, AP_Motors6DOF_M2, _motor_reverse[5], 1),

    // @Param: 7_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("7_DIRECTION", 7, AP_Motors6DOF_M2, _motor_reverse[6], 1),

    // @Param: 8_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("8_DIRECTION", 8, AP_Motors6DOF_M2, _motor_reverse[7], 1),

    // @Param: FV_CPLNG_K
    // @DisplayName: Forward/vertical to pitch decoupling factor
    // @Description: Used to decouple pitch from forward/vertical motion. 0 to disable, 1.2 normal
    // @Range: 0.0 1.5
    // @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("FV_CPLNG_K", 9, AP_Motors6DOF_M2, _forwardVerticalCouplingFactor, 1.0),

    // @Param: 9_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("9_DIRECTION", 10, AP_Motors6DOF_M2, _motor_reverse[8], 1),

    // @Param: 10_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("10_DIRECTION", 11, AP_Motors6DOF_M2, _motor_reverse[9], 1),

    // @Param: 11_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("11_DIRECTION", 12, AP_Motors6DOF_M2, _motor_reverse[10], 1),

    // @Param: 12_DIRECTION
    // @DisplayName: Motor normal or reverse
    // @Description: Used to change motor rotation directions without changing wires
    // @Values: 1:normal,-1:reverse
    // @User: Standard
    AP_GROUPINFO("12_DIRECTION", 13, AP_Motors6DOF_M2, _motor_reverse[11], 1),

    // @Param: 1_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("1_MAPPING", 14, AP_Motors6DOF_M2, _motor_mapping[0], 1),

    // @Param: 2_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("2_MAPPING", 15, AP_Motors6DOF_M2, _motor_mapping[1], 2),

    // @Param: 3_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("3_MAPPING", 16, AP_Motors6DOF_M2, _motor_mapping[2], 3),

    // @Param: 4_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("4_MAPPING", 17, AP_Motors6DOF_M2, _motor_mapping[3], 4),

    // @Param: 5_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("5_MAPPING", 18, AP_Motors6DOF_M2, _motor_mapping[4], 5),

    // @Param: 6_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("6_MAPPING", 19, AP_Motors6DOF_M2, _motor_mapping[5], 6),

    // @Param: 7_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("7_MAPPING", 20, AP_Motors6DOF_M2, _motor_mapping[6], 7),

    // @Param: 8_MAPPING
    // @DisplayName: Motor physical number
    // @Description: Used to mapping logic motor number to physic number
    // @Ragne: 1 8
    // @User: Standard
    AP_GROUPINFO("8_MAPPING", 21, AP_Motors6DOF_M2, _motor_mapping[7], 8),

    // @Param: CUSTOM_PITCH
    // @DisplayName: User corrected pitch
    // @Description: Used to correct pitch thr
    // @Ragne: 1 8
    // @User: Advanced
    AP_GROUPINFO("CUSTOM_PIT", 22, AP_Motors6DOF_M2, _custom_pitch_thr, 0.0f),

    // @Param: CUSTOM_ROLL
    // @DisplayName: User corrected roll
    // @Description: Used to correct roll thr
    // @Ragne: 1 8
    // @User: Advanced
    AP_GROUPINFO("CUSTOM_ROLL", 23, AP_Motors6DOF_M2, _custom_roll_thr, 0.0f),

    // @Param: FTP_FACT
    // @DisplayName: Forward throttle thrust correct factor
    // @Description: Used to correct forward thrust with throttle thrust of pitch
    // @Values: 1:multi with 1, -1:multi with -1
    // @User: Advanced
    AP_GROUPINFO("FTP_FACT", 24, AP_Motors6DOF_M2, _custom_thrust_factor[0], 1),

    // @Param: LTR_FACT
    // @DisplayName: Lateral throttle thrust correct factor
    // @Description: Used to correct Lateral thrust with throttle thrust of roll
    // @Values: 1:multi with 1, -1:multi with -1
    // @User: Advanced
    AP_GROUPINFO("LTR_FACT", 25, AP_Motors6DOF_M2, _custom_thrust_factor[1], -1),

    // @Param: TFP_FACT
    // @DisplayName: Throttle forward thrust correct factor
    // @Description: Used to correct Throttle thrust with forward thrust of pitch
    // @Values: 1:multi with 1, -1:multi with -1
    // @User: Advanced
    AP_GROUPINFO("TFP_FACT", 26, AP_Motors6DOF_M2, _custom_thrust_factor[2], -1),

    // @Param: TLR_FACT
    // @DisplayName: Throttle lateral thrust correct factor
    // @Description: Used to correct Throttle thrust with lateral thrust of roll
    // @Values: 1:multi with 1, -1:multi with -1
    // @User: Advanced
    AP_GROUPINFO("TLR_FACT", 27, AP_Motors6DOF_M2, _custom_thrust_factor[3], 1),

    // @Param: CFT
    // @DisplayName: User corrected forward thrust
    // @Description: Used to correct forward thrust
    // @Ragne: 1 8
    // @User: Advanced
    AP_GROUPINFO("CFT", 28, AP_Motors6DOF_M2, _custom_forward_thrust, 0.0f),

    // @Param: THR_RATIO
    // @DisplayName: Negative thrust ratio
    // @Description: Used to correct Negative thrust ratio
    // @Values: !=0: use ratio, 0: donot use ratio
    // @User: Advanced
    AP_GROUPINFO("THR_RATIO", 29, AP_Motors6DOF_M2, _custom_negative_thrust_ratio, 1.0f),

    AP_GROUPEND
};

static void to_designer_order_out(float out[]);
static void motor_vector_force_debug(float rpy_out[], float linear_out[], float rpyt_out[], float ratio);

void AP_Motors6DOF_M2::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }

    // hard coded config for supported frames
    switch ((sub_frame_t)frame_class) {
        //                 Motor #              Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor  Testing Order
    case SUB_FRAME_CUSTOM:
        // Put your custom motor setup here
        add_motor_raw_6dof(AP_MOTORS_MOT_1,    -1.0f,           1.0f,           1.0f,           1.0f,             -1.0f,                1.0f,           1);
        add_motor_raw_6dof(AP_MOTORS_MOT_2,    -1.0f,          -1.0f,           1.0f,          -1.0f,              1.0f,                1.0f,           2);
        add_motor_raw_6dof(AP_MOTORS_MOT_3,    -1.0f,          -1.0f,          -1.0f,          -1.0f,             -1.0f,               -1.0f,           3);
        add_motor_raw_6dof(AP_MOTORS_MOT_4,    -1.0f,           1.0f,          -1.0f,           1.0f,              1.0f,               -1.0f,           4);
        add_motor_raw_6dof(AP_MOTORS_MOT_5,     1.0f,          -1.0f,          -1.0f,           1.0f,             -1.0f,                1.0f,           5);
        add_motor_raw_6dof(AP_MOTORS_MOT_6,     1.0f,          -1.0f,           1.0f,           1.0f,              1.0f,               -1.0f,           6);
        add_motor_raw_6dof(AP_MOTORS_MOT_7,     1.0f,           1.0f,          -1.0f,          -1.0f,              1.0f,                1.0f,           7);
        add_motor_raw_6dof(AP_MOTORS_MOT_8,     1.0f,           1.0f,           1.0f,          -1.0f,             -1.0f,               -1.0f,           8);
        break;
    default:
        break;
    }
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
// TODO pull code that is common to output_armed_not_stabilizing into helper functions
// ToDo calculate headroom for rpy to be added for stabilization during full throttle/forward/lateral commands
extern bool is_param_print(void);
extern bool is_dbg_motor;
extern float correct_pitch_thr;
extern float correct_roll_thr;
void AP_Motors6DOF_M2::output_armed_stabilizing()
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

    float corrected_pitch = _custom_pitch_thr + _pitch_thr;
    float corrected_roll = _custom_roll_thr + _roll_thr;

    // test real data
    // roll_thrust = 0.04;
    // pitch_thrust = 0.594;
    // yaw_thrust = -0.001;
    // throttle_thrust = 0.284;
    // corrected_pitch = -0.02;
    // corrected_roll = 0;


    if (is_param_print() && is_dbg_motor) {
        printf("============================\r\n");
        printf("pitch %3.4f/%3.1f roll %3.4f/%3.1f\r\n", _pitch_thr, ToDeg(_pitch_thr),  _roll_thr, ToDeg(_roll_thr));
        printf("custom: pitch %3.4f/%3.1f roll %3.4f/%3.1f\r\n", _custom_pitch_thr.get(), ToDeg(_custom_pitch_thr.get()),
            _custom_roll_thr.get(), ToDeg(_custom_roll_thr.get()));
        printf("corrected: pitch %3.4f/%3.1f roll %3.4f/%3.1f\r\n", corrected_pitch, ToDeg(corrected_pitch), 
            corrected_roll, ToDeg(corrected_roll));
        printf("\r\n");
        printf("thrust: roll %2.4f pitch %2.4f yaw %2.4f\r\n", roll_thrust, pitch_thrust, yaw_thrust);
        printf("thurst: forward = %2.4f lateral = %2.4f throttle = %2.4f\r\n", forward_thrust, lateral_thrust, throttle_thrust);
        printf("thrust custom: forward %2.4f\r\n", _custom_forward_thrust.get());
    }

    // _custom_thrust_factor can be derived from
    // forward thrust, front is +
    // lateral thrust, right is +
    // throttle thrust, up is +
    // so
    // if we need a NED throttle thrust named desired_throttle, it should be up or down
    //   throttle = desired_throttle * cos(pitch) * cos(roll)
    //   and compensate
    //   forward = desired_throttle * sin(pitch)
    //   lateral = -desired_throttle * sin(phi)
    // if we need a NED forward thrust named desired_forward, it should be front or back
    //   forward = -desired_forward * cos(pitch)
    //   and compensate
    //   throttle = - desired_forward * sin(pitch)
    // if we need a NED lateral thrust named desired_lateral, it should be left or right
    //   lateral = desired_lateral * cos(phi)
    //   and compensate
    //   throttle = desired_lateral * sin(phi)

    forward_thrust = forward_thrust * cosf(corrected_pitch) 
                    + _custom_thrust_factor[0] * throttle_thrust * sinf(corrected_pitch)
                    + _custom_forward_thrust;
    lateral_thrust = lateral_thrust * cosf(corrected_roll) 
                    + _custom_thrust_factor[1] * throttle_thrust * sinf(corrected_roll);
    throttle_thrust = throttle_thrust * cosf(corrected_pitch) * cosf(corrected_roll) 
                    + _custom_thrust_factor[2] * _forward_in * sinf(corrected_pitch) 
                    + _custom_thrust_factor[3] * _lateral_in * sinf(corrected_roll);

    if (is_param_print() && is_dbg_motor) {
        printf("thrust corrected: forward %2.4f lateral %2.4f throttle %2.4f\r\n", forward_thrust, lateral_thrust, throttle_thrust);
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
    int motor_lr_factor[8] = {-1,  1, -1,  1,  1, -1, -1,  1}; // L/R propeller factor, to make positive thrust with positive sign
    float thrust_rpyt_out_max = 1;
    if (!is_zero(_custom_negative_thrust_ratio)) {
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                _thrust_rpyt_out[i] = rpy_out[i] + linear_out[i];
                if ((_thrust_rpyt_out[i] < 0) != (motor_lr_factor[i] < 0)) {
                    _thrust_rpyt_out[i] = _thrust_rpyt_out[i] / _custom_negative_thrust_ratio;
                }
                if (fabsf(_thrust_rpyt_out[i]) > thrust_rpyt_out_max) {
                    thrust_rpyt_out_max = fabsf(_thrust_rpyt_out[i]);
                }
            }
        }

        if (is_param_print() && is_dbg_motor) {
            float ratio_rpyt_out[8];
            for (i = 0; i < 8; i++) {
                ratio_rpyt_out[i] = _thrust_rpyt_out[i];
            }

            to_designer_order_out(ratio_rpyt_out);
            
            printf("\r\nratio rpyt out:\r\n");
            for (i = 0; i < 8; i++) {
                printf("%2.4f ", ratio_rpyt_out[i]);
            }
        }

        // normalize
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                _thrust_rpyt_out[i] = constrain_float(_thrust_rpyt_out[i] / thrust_rpyt_out_max,-1.0f,1.0f);
            }
        }

        if (is_param_print() && is_dbg_motor) {
            motor_vector_force_debug(rpy_out, linear_out, _thrust_rpyt_out, _custom_negative_thrust_ratio);
        }

        // thurst analize should not effected by direction
        // motor direction
        for (i=0; i<AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                _thrust_rpyt_out[i] = constrain_float(_motor_reverse[i] * _thrust_rpyt_out[i],-1.0f,1.0f);
            }
        }
    } else {
        if (is_param_print() && is_dbg_motor) {
            motor_vector_force_debug(rpy_out, linear_out, _thrust_rpyt_out, _custom_negative_thrust_ratio);
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
