/// @file	AP_Motors6DOF.h
/// @brief	Motor control class for ROVs with direct control over 6DOF (or fewer) in movement

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_Motors6DOF.h"

/// @class      AP_MotorsMatrix
class AP_Motors6DOF_M2 : public AP_Motors6DOF {
public:

    AP_Motors6DOF_M2(uint16_t loop_rate, uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_Motors6DOF(loop_rate, speed_hz) {
        AP_Param::setup_object_defaults(this, var_info);
    };

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo        var_info[];

    // Override parent
    void setup_motors(motor_frame_class frame_class, motor_frame_type frame_type) override;

    //
    // void set_roll_pitch_thr(float roll, float pitch) { _roll_thr = roll; _pitch_thr = pitch; }

protected:

    void output_armed_stabilizing() override;

    // Parameters
    AP_Int8             _motor_mapping[AP_MOTORS_MAX_NUM_MOTORS];
    
    AP_Float            _custom_pitch_thr;
    AP_Float            _custom_roll_thr;
    AP_Int8             _custom_thrust_factor[4];
    AP_Float            _custom_forward_thrust;
    AP_Float            _custom_negative_thrust_ratio;
};
