/// @file	AP_Motors6DOF.h
/// @brief	Motor control class for ROVs with direct control over 6DOF (or fewer) in movement

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"

/// @class      AP_MotorsMatrix
class AP_Motors6DOF : public AP_MotorsMatrix {
public:

    AP_Motors6DOF(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMatrix(loop_rate, speed_hz) {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // Supported frame types
    typedef enum {
        SUB_FRAME_BLUEROV1,
        SUB_FRAME_VECTORED,
        SUB_FRAME_VECTORED_6DOF,
        SUB_FRAME_VECTORED_6DOF_90DEG,
        SUB_FRAME_SIMPLEROV_3,
        SUB_FRAME_SIMPLEROV_4,
        SUB_FRAME_SIMPLEROV_5,
        SUB_FRAME_CUSTOM
    } sub_frame_t;

    // Override parent
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // Override parent
    void output_min() override;

    // Map thrust input -1~1 to pwm output 1100~1900
    int16_t calc_thrust_to_pwm(float thrust_in) const;

    // output_to_motors - sends minimum values out to the motors
    void output_to_motors() override;

    // returns a vector with roll, pitch, and yaw contributions
    Vector3f get_motor_angular_factors(int motor_number);

    // returns true if motor is enabled
    bool motor_is_enabled(int motor_number);

    bool set_reversed(int motor_number, bool reversed);

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

    //Override MotorsMatrix method
    void add_motor_raw_6dof(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, float climb_fac, float forward_fac, float lat_fac, uint8_t testing_order);
    
    // custom

    // used to calc thurst decomposition
    FUNCTOR_TYPEDEF(thrust_decomposition_fn_t, Vector3f, Vector3f&, Vector3f, float); // forward lateral throttle
    void                set_thrust_decomposition_callback(thrust_decomposition_fn_t callback) {
        _thrust_decomposition_callback = callback;
    }

    FUNCTOR_TYPEDEF(setup_custom_motors_fn_t, void); // forward lateral throttle
    void                set_setup_custom_motors_callback(setup_custom_motors_fn_t callback) {
        _setup_custom_motors_callback = callback;
    }

    FUNCTOR_TYPEDEF(motor_fn_t, void, int16_t *, uint8_t); // forward lateral throttle
    void                set_motor_callback(motor_fn_t callback) {
        _motor_callback = callback;
    }

    void set_motors_log(bool log_start) {motor_log_start = log_start;}

protected:
    // return current_limit as a number from 0 ~ 1 in the range throttle_min to throttle_max
    float               get_current_limit_max_throttle() override;

    void output_armed_stabilizing() override;
    void output_armed_stabilizing_vectored();
    void output_armed_stabilizing_vectored_6dof();

    // Parameters
    AP_Int8             _motor_reverse[AP_MOTORS_MAX_NUM_MOTORS];
    AP_Float            _forwardVerticalCouplingFactor;

    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to throttle (climb/descent)
    float               _forward_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to forward/backward
    float               _lateral_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to lateral (left/right)

    // current limiting
    float _output_limited = 1.0f;
    float _batt_current_last = 0.0f;

    // custom
    
    AP_Int8                   _motor_mapping[AP_MOTORS_MAX_NUM_MOTORS];

    bool motor_log_start = false;

    thrust_decomposition_fn_t _thrust_decomposition_callback;
    setup_custom_motors_fn_t  _setup_custom_motors_callback;
    motor_fn_t _motor_callback;

    void output_armed_stabilizing_custom();

    // array of motor output values
    float _actuator[AP_MOTORS_MAX_NUM_MOTORS];
};