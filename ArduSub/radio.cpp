#include "Sub.h"

void Sub::init_rc_in()
{
    channel_pitch    = RC_Channels::rc_channel(RC_IN_CHANNEL_PITCH);
    channel_roll     = RC_Channels::rc_channel(RC_IN_CHANNEL_ROLL);
    channel_throttle = RC_Channels::rc_channel(RC_IN_CHANNEL_THROTTLE);
    channel_yaw      = RC_Channels::rc_channel(RC_IN_CHANNEL_YAW);
    channel_forward  = RC_Channels::rc_channel(RC_IN_CHANNEL_FORWARD);
    channel_lateral  = RC_Channels::rc_channel(RC_IN_CHANNEL_LATERAL);

    /*channel_left_pump = RC_Channels::rc_channel(RC_IN_CHANNEL_LEFT_PUMP);
    channel_right_pump = RC_Channels::rc_channel(RC_IN_CHANNEL_RIGHT_PUMP);
    channel_up_pump = RC_Channels::rc_channel(RC_IN_CHANNEL_UP_PUMP);
    channel_arm = RC_Channels::rc_channel(RC_IN_CHANNEL_ARM);*/

    // set rc channel ranges
    //channel_roll->set_angle(ROLL_PITCH_INPUT_MAX);
    //channel_pitch->set_angle(ROLL_PITCH_INPUT_MAX);
    channel_yaw->set_angle(ROLL_PITCH_INPUT_MAX);
    //channel_throttle->set_range(1000);
    //channel_forward->set_angle(ROLL_PITCH_INPUT_MAX);
    //channel_lateral->set_angle(ROLL_PITCH_INPUT_MAX);

    for (int i = 0; i < 16; i++) {
        RC_Channels::set_override(i, 1500);
    }
    
#if 0
    // set default dead zones
    channel_roll->set_default_dead_zone(30);
    channel_pitch->set_default_dead_zone(30);
    channel_throttle->set_default_dead_zone(30);
    channel_yaw->set_default_dead_zone(40);
    channel_forward->set_default_dead_zone(30);
    channel_lateral->set_default_dead_zone(30);

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    // initialize rc input to 1500 on control channels (rather than 0)
    for (int i = 0; i < 6; i++) {
        RC_Channels::set_override(i, 1500);
    }

    RC_Channels::set_override(6, 1500); // camera pan channel
    RC_Channels::set_override(7, 1500); // camera tilt channel

    RC_Channel* chan = RC_Channels::rc_channel(8);
    uint16_t min = chan->get_radio_min();
    RC_Channels::set_override(8, min); // lights 1 channel

    chan = RC_Channels::rc_channel(9);
    min = chan->get_radio_min();
    RC_Channels::set_override(9, min); // lights 2 channel

    RC_Channels::set_override(10, 1100); // video switch
#endif
#endif

}

// init_rc_out -- initialise motors and check if pilot wants to perform ESC calibration
void Sub::init_rc_out()
{
    motors.set_update_rate(g.rc_speed);
    motors.set_loop_rate(scheduler.get_loop_rate_hz());
    motors.init((AP_Motors::motor_frame_class)g.frame_configuration.get(), AP_Motors::motor_frame_type::MOTOR_FRAME_TYPE_PLUS);
    motors.set_throttle_range(-2200, 2200);

    // enable output to motors
    if (arming.rc_calibration_checks(true)) {
        enable_motor_output();
    }

    // refresh auxiliary channel to function map
    SRV_Channels::update_aux_servo_function();

    /*SRV_Channels::set_aux_channel_default(SRV_Channel::k_steering, AP_MOTORS_MOT_3);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleLeft, AP_MOTORS_MOT_4);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_throttleRight, AP_MOTORS_MOT_5);
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_boost_throttle, AP_MOTORS_MOT_6);

    SRV_Channels::set_output_pwm(SRV_Channel::k_steering, channel_arm->get_radio_trim());
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, channel_left_pump->get_radio_trim());
    SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, channel_right_pump->get_radio_trim());
    SRV_Channels::set_output_pwm(SRV_Channel::k_boost_throttle, channel_up_pump->get_radio_trim());
    */
}
