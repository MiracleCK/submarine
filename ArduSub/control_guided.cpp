#include "Sub.h"

#define GUIDED_WP_RADIUS     100		// default guided waypoint radius in cm

// guided_init - initialise guided controller
bool Sub::guided_init(bool ignore_checks)
{
    if (!position_ok() && !ignore_checks) {
        return false;
    }

	const Vector3f& curr_pos = inertial_nav.get_position();
    pos_control.set_xy_target(curr_pos.x, curr_pos.y);

    // initialise pos controller speed, acceleration
    pos_control.set_max_speed_xy(POSCONTROL_SPEED);
    pos_control.set_max_accel_xy(POSCONTROL_ACCEL_XY);

    // set vehicle velocity and acceleration to zero
    pos_control.set_desired_velocity_xy(0.0f, 0.0f);
    pos_control.set_desired_accel_xy(0.0f, 0.0f);

    //pos_control.set_leash_length_xy(500.0f);

    // initialise position controller if not already active
    if (!pos_control.is_active_xy()) {
        pos_control.init_xy_controller();
    }

    is_waypoint_running = false;
    is_reached_destination = false;
    is_wp_destination_valid = false;

    printf("guided mode! \r\n");
    return true;
}


// sets guided mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and guided waypoint is outside the fence
bool Sub::guided_set_destination(const Location& dest_loc)
{
#if AC_FENCE == ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    if (!fence.check_destination_within_fence(dest_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif

    Vector2f res_vec;
    if (!dest_loc.get_vector_xy_from_origin_NE(res_vec)) {
        return false;
    }

    // convert altitude
    wp_destination.x = res_vec.x;
    wp_destination.y = res_vec.y;
    wp_destination.z = 0;

    pos_control.set_xy_target(wp_destination.x, wp_destination.y);
    printf("guided_set_destination %d %d %d\r\n", dest_loc.lat, dest_loc.lng, dest_loc.alt);

	is_reached_destination = false;
    is_waypoint_running = true;

    // log target
    Log_Write_GuidedTarget(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void Sub::guided_run()
{
	// if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        return;
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
    
    Vector3f curr_pos = inertial_nav.get_position();
	curr_pos.z = 0;
	Vector3f dist_to_dest = curr_pos - wp_destination;
    if( dist_to_dest.length() <= GUIDED_WP_RADIUS ) {
        is_reached_destination = true;
    }
    
    if (is_reached_destination) {
        if (is_waypoint_running == true) {
            printf("reached destination\r\n");
            gcs().send_mission_item_reached_message(1);
            is_waypoint_running = false;
            is_reached_destination = false;
            is_mode_auto_switch_enabled = true;
            is_wp_destination_valid = true;

            // switch to manual then auto select a mode
            sub.set_mode(MANUAL, ModeReason::GUIDED_DONE);
        }
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
    }

    // run waypoint controller
    //pos_control.set_leash_length_xy(500.0f);
    pos_control.update_xy_controller();
    //pos_control.update_z_controller();

    float lateral_out, forward_out;
    translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

	// convert pilot input to lean angles
	// To-Do: convert get_pilot_desired_lean_angles to return angles as floats
	float target_roll, target_pitch;
	get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // call attitude controller
    if (!is_zero(target_yaw_rate)) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    } else {
    	if( dist_to_dest.length() > 200 ) {
    	//if(1) {
    		target_yaw_rate = get_bearing_cd(curr_pos, wp_destination);
    		printf("target_yaw_rate %f\r\n", target_yaw_rate);
    	} else {
			target_yaw_rate = ahrs.yaw_sensor;
    	}

    	if (0) {
	    	static uint32_t _startup_ms = 0;

	        if(_startup_ms == 0) {
				_startup_ms = AP_HAL::millis();
	        }

	        if(AP_HAL::millis() - _startup_ms > 1000) {
				_startup_ms = AP_HAL::millis();
				
	            //hal.shell->
	            printf("target_yaw_rate %f\r\n",
	            				target_yaw_rate);
		    }
		}
		
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll, target_pitch, target_yaw_rate, true);
    }
}


// Guided Limit code
struct Guided_Limit {
    uint32_t timeout_ms;  // timeout (in seconds) from the time that guided is invoked
    float alt_min_cm;   // lower altitude limit in cm above home (0 = no limit)
    float alt_max_cm;   // upper altitude limit in cm above home (0 = no limit)
    float horiz_max_cm; // horizontal position limit in cm from where guided mode was initiated (0 = no limit)
    uint32_t start_time;// system time in milliseconds that control was handed to the external computer
    Vector3f start_pos; // start position as a distance from home in cm.  used for checking horiz_max limit
} guided_limit;

// guided_limit_clear - clear/turn off guided limits
void Sub::guided_limit_clear()
{
    guided_limit.timeout_ms = 0;
    guided_limit.alt_min_cm = 0.0f;
    guided_limit.alt_max_cm = 0.0f;
    guided_limit.horiz_max_cm = 0.0f;
}

// guided_limit_set - set guided timeout and movement limits
void Sub::guided_limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm)
{
    guided_limit.timeout_ms = timeout_ms;
    guided_limit.alt_min_cm = alt_min_cm;
    guided_limit.alt_max_cm = alt_max_cm;
    guided_limit.horiz_max_cm = horiz_max_cm;
}

// guided_limit_init_time_and_pos - initialise guided start time and position as reference for limit checking
//  only called from AUTO mode's auto_nav_guided_start function
void Sub::guided_limit_init_time_and_pos()
{
    // initialise start time
    guided_limit.start_time = AP_HAL::millis();

    // initialise start position from current position
    guided_limit.start_pos = inertial_nav.get_position();
}

// guided_limit_check - returns true if guided mode has breached a limit
//  used when guided is invoked from the NAV_GUIDED_ENABLE mission command
bool Sub::guided_limit_check()
{
    // check if we have passed the timeout
    if ((guided_limit.timeout_ms > 0) && (AP_HAL::millis() - guided_limit.start_time >= guided_limit.timeout_ms)) {
        return true;
    }

    // get current location
    const Vector3f& curr_pos = inertial_nav.get_position();

    // check if we have gone below min alt
    if (!is_zero(guided_limit.alt_min_cm) && (curr_pos.z < guided_limit.alt_min_cm)) {
        return true;
    }

    // check if we have gone above max alt
    if (!is_zero(guided_limit.alt_max_cm) && (curr_pos.z > guided_limit.alt_max_cm)) {
        return true;
    }

    // check if we have gone beyond horizontal limit
    if (guided_limit.horiz_max_cm > 0.0f) {
        float horiz_move = get_horizontal_distance_cm(guided_limit.start_pos, curr_pos);
        if (horiz_move > guided_limit.horiz_max_cm) {
            return true;
        }
    }

    // if we got this far we must be within limits
    return false;
}

