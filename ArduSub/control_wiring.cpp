#include "Sub.h"

// wiring_init - initialise wiring controller
bool Sub::wiring_init()
{
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);
    last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

// wiring_run - runs the main wiring controller
// should be called at 100hz or more
void Sub::wiring_run()
{
    uint32_t tnow = AP_HAL::millis();
    float target_roll, target_pitch;
    float target_yaw_rate;

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed() ||
        (channel_yaw->get_radio_in() == 1500 &&
	    channel_forward->get_radio_in() == 1500 &&
	    channel_lateral->get_radio_in() == 1500)) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    if(0) {
        static uint32_t _startup_ms = 0;

        if(_startup_ms == 0) {
			_startup_ms = AP_HAL::millis();
        }

        if(AP_HAL::millis() - _startup_ms > 1000) {
			_startup_ms = AP_HAL::millis();
			
		    printf("yaw %d\r\n", channel_yaw->get_radio_in());
		    printf("forward %d\r\n", channel_forward->get_radio_in());
		    printf("lateral %d\r\n", channel_lateral->get_radio_in());
		    printf("\r\n");
	    }
	}

	motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
    // update attitude controller targets

    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        last_pilot_heading = ahrs.yaw_sensor;
        last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0;  // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true);
        }
    }

    // output pilot's throttle
    attitude_control.set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);

    //control_in is range -1000-1000
    //radio_in is raw pwm value
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}