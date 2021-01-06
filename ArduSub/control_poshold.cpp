// ArduSub position hold flight mode
// GPS required
// Jacob Walser August 2016

#include "Sub.h"

#if POSHOLD_ENABLED == ENABLED

/// init_target to a position in cm from ekf origin
void Sub::poshold_init_target(void)
{
	const Vector3f& curr_pos = inertial_nav.get_position();
	//const Vector3f& curr_vel = inertial_nav.get_velocity();

    // initialise pos controller speed, acceleration
    pos_control.set_max_speed_xy(POSCONTROL_SPEED);
    pos_control.set_max_accel_xy(POSCONTROL_ACCEL_XY);

    // set target position
    pos_control.set_xy_target(curr_pos.x, curr_pos.y);

    // set vehicle velocity and acceleration to zero
    //pos_control.set_desired_velocity_xy(curr_vel.x, curr_vel.y);
    pos_control.set_desired_velocity_xy(0.0f,0.0f);
    pos_control.set_desired_accel_xy(0.0f,0.0f);

    pos_control.set_leash_length_xy(500.0f);

    // initialise position controller if not already active
    if (!pos_control.is_active_xy()) {
        pos_control.init_xy_controller();
    }
}

bool Sub::poshold_position_ok()
{
    // with EKF use filter status and ekf check
    nav_filter_status filt_status = inertial_nav.get_filter_status();

    return (filt_status.flags.using_gps && \
            !filt_status.flags.gps_glitching && \
            filt_status.flags.gps_quality_good);
}

// poshold_init - initialise PosHold controller
bool Sub::poshold_init()
{
    // fail to initialise PosHold mode if no GPS lock
    if (!position_ok() || !poshold_position_ok()) {
        return false;
    }

	// initialize vertical speeds and acceleration
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());
	
    // initialize controller
    poshold_init_target();
    
    last_pilot_heading = ahrs.yaw_sensor;

    return true;
}

extern uint8_t pos_get_flag;
extern Location target_loc;

// poshold_run - runs the PosHold controller
// should be called at 100hz or more
void Sub::poshold_run()
{
    if (smart_mode_auto_switch()) {
        return;
    }
    
    if (position_ok()) {
        if (pos_get_flag == true) {
            printf("flag = %d \r\n", pos_get_flag);
            ahrs.get_location(target_loc);
            printf("alt lng lat = %4d %4d %4d \r\n",  target_loc.alt, target_loc.lng, target_loc.lat);
            pos_get_flag = false;
            printf("wp get success! \r\n");
        }
    }
    
    uint32_t tnow = AP_HAL::millis();

    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        poshold_init_target();
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    ///////////////////////
    // update xy outputs //
    float pilot_lateral = channel_lateral->norm_input();
    float pilot_forward = channel_forward->norm_input();

    float lateral_out = 0;
    float forward_out = 0;

    // Allow pilot to reposition the sub
    if (fabsf(pilot_lateral) > 0.1 || fabsf(pilot_forward) > 0.1) {
        lateral_out = pilot_lateral;
        forward_out = pilot_forward;
        poshold_init_target();
    } else {
    	// run posxy controller
    	pos_control.update_xy_controller();
    	
        translate_wpnav_rp(lateral_out, forward_out);
    }

    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    /////////////////////
    // Update attitude //

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // update attitude controller targets
    if (!is_zero(target_yaw_rate)) { // call attitude controller with rate yaw determined by pilot input
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
        last_pilot_heading = ahrs.yaw_sensor;
        last_pilot_yaw_input_ms = tnow; // time when pilot last changed heading

    } else { // hold current heading

        // this check is required to prevent bounce back after very fast yaw maneuvers
        // the inertia of the vehicle causes the heading to move slightly past the point when pilot input actually stopped
        if (tnow < last_pilot_yaw_input_ms + 250) { // give 250ms to slow down, then set target heading
            target_yaw_rate = 0; // Stop rotation on yaw axis

            // call attitude controller with target yaw rate = 0 to decelerate on yaw axis
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
            last_pilot_heading = ahrs.yaw_sensor; // update heading to hold

        } else { // call attitude controller holding absolute absolute bearing
            attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, last_pilot_heading, true);
        }
    }

    ///////////////////
    // Update z axis //

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // call z axis position controller
    if (ap.at_bottom) {
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover()); // clear velocity and position targets, and integrator
        pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
    } else {
        pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    }

    pos_control.update_z_controller();
}
#endif  // POSHOLD_ENABLED == ENABLED
