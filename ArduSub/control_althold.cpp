#include "Sub.h"


bool use_angle_rate = true;

/*
 * control_althold.pde - init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool Sub::althold_init()
{
    if(!control_check_barometer()) {
        return false;
    }

    // initialize vertical speeds and leash lengths
    // sets the maximum speed up and down returned by position controller
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    last_pilot_heading = ahrs.yaw_sensor;
    last_pilot_pitch = ahrs.pitch_sensor;
    last_pilot_roll = ahrs.roll_sensor;

    is_z_ctrl_relaxed = false;
    engageStopZ = false;
    lastVelocityZWasNegative = is_negative(inertial_nav.get_velocity_z());;

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::althold_run()
{
    if (smart_mode_auto_switch()) {
        return;
    }

    if (use_angle_rate) {
        althold_run_rate();
        return;
    }

    uint32_t tnow = AP_HAL::millis();

    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());
        last_pilot_heading = ahrs.yaw_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot desired lean angles
    float target_roll, target_pitch;

    // Check if set_attitude_target_no_gps is valid
    if (tnow - sub.set_attitude_target_no_gps.last_message_ms < 5000) {
        float target_yaw;
        Quaternion(
            set_attitude_target_no_gps.packet.q
        ).to_euler(
            target_roll,
            target_pitch,
            target_yaw
        );
        target_roll = degrees(target_roll);
        target_pitch = degrees(target_pitch);
        target_yaw = degrees(target_yaw);

        attitude_control.input_euler_angle_roll_pitch_yaw(target_roll * 1e2f, target_pitch * 1e2f, target_yaw * 1e2f, true);
        return;
    }

    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // call attitude controller
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

    // Hold actual position until zero derivative is detected
    // static bool engageStopZ = true;
    // Get last user velocity direction to check for zero derivative points
    // static bool lastVelocityZWasNegative = false;
    if (fabsf(channel_throttle->norm_input()-0.5f) > 0.05f) { // Throttle input above 5%
        // output pilot's throttle
        attitude_control.set_throttle_out(channel_throttle->norm_input(), false, g.throttle_filt);
        // reset z targets to current values
        pos_control.relax_alt_hold_controllers();
        engageStopZ = true;
        lastVelocityZWasNegative = is_negative(inertial_nav.get_velocity_z());
    } else { // hold z

        if (ap.at_bottom) {
            pos_control.relax_alt_hold_controllers(); // clear velocity and position targets
            pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
        }

        // Detects a zero derivative
        // When detected, move the altitude set point to the actual position
        // This will avoid any problem related to joystick delays
        // or smaller input signals
        if(engageStopZ && (lastVelocityZWasNegative ^ is_negative(inertial_nav.get_velocity_z()))) {
            engageStopZ = false;
            pos_control.relax_alt_hold_controllers();
        }

        pos_control.update_z_controller();
    }

    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());
}

void Sub::get_alt_hold_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out)
{
    Vector3f rate_ef_level, rate_bf_level, rate_bf_request;

    // apply circular limit to pitch and roll inputs
    float total_in = norm(pitch_in, roll_in);

    if (total_in > ROLL_PITCH_INPUT_MAX) {
        float ratio = (float)ROLL_PITCH_INPUT_MAX / total_in;
        roll_in *= ratio;
        pitch_in *= ratio;
    }

    // calculate roll, pitch rate requests
    if (g.acro_expo <= 0) {
        rate_bf_request.x = roll_in * g.acro_rp_p;
        rate_bf_request.y = pitch_in * g.acro_rp_p;
    } else {
        // expo variables
        float rp_in, rp_in3, rp_out;

        // range check expo
        if (g.acro_expo > 1.0f) {
            g.acro_expo = 1.0f;
        }

        // roll expo
        rp_in = float(roll_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.x = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;

        // pitch expo
        rp_in = float(pitch_in)/ROLL_PITCH_INPUT_MAX;
        rp_in3 = rp_in*rp_in*rp_in;
        rp_out = (g.acro_expo * rp_in3) + ((1 - g.acro_expo) * rp_in);
        rate_bf_request.y = ROLL_PITCH_INPUT_MAX * rp_out * g.acro_rp_p;
    }

    // calculate yaw rate request
    rate_bf_request.z = yaw_in * g.acro_yaw_p;

    // hand back rate request
    roll_out = rate_bf_request.x;
    pitch_out = rate_bf_request.y;
    yaw_out = rate_bf_request.z;
}

bool Sub::attitude_control_rate(bool is_reset, int16_t roll, int16_t pitch, int16_t yaw) {
    static bool is_reseting = false;

    // get pilot desired lean angles
    float target_roll_rate, target_pitch_rate, target_yaw_rate;

    get_alt_hold_pilot_desired_angle_rates(
        roll, pitch, yaw, 
        target_roll_rate, target_pitch_rate, target_yaw_rate);

    if (is_reset) {
        is_reseting = true;
        target_pitch_rate = 0.0f;
        target_roll_rate = 0.0f;
    }

    if (!is_zero(target_roll_rate) || !is_zero(target_pitch_rate)) {
        is_reseting = false;
    }

    if (is_reseting) {
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0.0f, 0.0f, target_yaw_rate);;
    } else if (is_ned_pilot) {
        target_roll_rate *= 50;
        target_pitch_rate *= 50;
        attitude_control.input_euler_rate_roll_limited_pitch_yaw(target_roll_rate, target_pitch_rate, target_yaw_rate);
        // attitude_control.input_euler_rate_roll0_pitch_yaw(target_pitch_rate, target_yaw_rate);
        // attitude_control.input_euler_rate_roll_pitch_yaw(target_roll_rate, target_pitch_rate, target_yaw_rate);
    } else {
        attitude_control.input_rate_bf_roll_pitch_yaw(target_roll_rate, target_pitch_rate, target_yaw_rate);
    }

    return !is_zero(target_yaw_rate) || !is_zero(target_roll_rate) || !is_zero(target_pitch_rate);
}

void Sub::althold_run_rate()
{
    // initialize vertical speeds and acceleration
    pos_control.set_max_speed_z(-get_pilot_speed_dn(), g.pilot_speed_up);
    pos_control.set_max_accel_z(g.pilot_accel_z);

    if(is_ned_pilot) {
    	distance_control.update_distance();
    }

    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when not auto-armed (i.e. on the ground, pilot has never raised throttle)
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        pos_control.relax_alt_hold_controllers(motors.get_throttle_hover());

        if(distance_control.bottom_face_is_active()) {
	    	distance_control.relax_z_controller((float)distance_control.get_bottom_cm());
	    } else if(distance_control.top_face_is_active()) {
	    	distance_control.relax_z_controller((float)distance_control.get_top_cm());
	    } else if(distance_control.front_face_is_active()) {
	    	distance_control.relax_x_controller((float)distance_control.get_front_cm());
	    } else if(distance_control.back_face_is_active()) {
	    	distance_control.relax_x_controller((float)distance_control.get_back_cm());
	    } else if(distance_control.right_face_is_active()) {
	    	distance_control.relax_y_controller((float)distance_control.get_right_cm());
	    } else if(distance_control.left_face_is_active()) {
	    	distance_control.relax_y_controller((float)distance_control.get_left_cm());
	    }

        is_z_ctrl_relaxed = false;
        engageStopZ = false;
        lastVelocityZWasNegative = is_negative(inertial_nav.get_velocity_z());
        is_request_reset_rp = true; // to reset rp when arm
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    attitude_control_rate(is_request_reset_rp, 
        channel_roll->get_control_in(), channel_pitch->get_control_in(), channel_yaw->get_control_in());
    if (is_request_reset_rp) {
        is_request_reset_rp = false;
    }

    uint32_t tnow = AP_HAL::millis();
	
	if(	is_ned_pilot &&
		((distance_control.bottom_face_is_active() && 
		(distance_control.get_bottom_cm() > 5 && distance_control.get_bottom_cm() < 200)) ||
	    (distance_control.top_face_is_active() && 
	    (distance_control.get_top_cm() < -5 && distance_control.get_top_cm() > -200)))) {
	    int16_t distance;
	    if(distance_control.bottom_face_is_active()) {
	    	distance = distance_control.get_bottom_cm();
	    } else {
	    	distance = distance_control.get_top_cm();
	    }
	    	
		if (fabsf(pilot_trans_thrusts.z) > 0.05f) {
	        // output pilot's throttle
	        motors.set_throttle_pilot(pilot_trans_thrusts.z);
	        attitude_control.set_throttle_out(motors.get_throttle_hover(), false, g.throttle_filt);
	        distance_control.relax_z_controller((float)distance);
	        pos_control.relax_alt_hold_controllers();
	        last_pilot_z_input_ms = tnow;
	        is_z_ctrl_relaxed = true;
	    } else { // hold z
	        if (tnow - last_pilot_z_input_ms > 500) {
	        	if(is_z_ctrl_relaxed) {
		        	distance_control.relax_z_controller((float)distance);
		        	is_z_ctrl_relaxed = false;
	        	}
		    	distance_control.update_z_controller((float)distance);
	        } else {
				motors.set_throttle_pilot(0);
				attitude_control.set_throttle_out(motors.get_throttle_hover(), false, g.throttle_filt); 
	        }
	        pos_control.relax_alt_hold_controllers();
	    }
	} else {
		if(is_ned_pilot && distance_control.limit_enable()) {
			if((pilot_trans_thrusts.z < -0.0f && distance_control.get_bottom_limit_cm() != 0 && 
				distance_control.get_bottom_cm() < distance_control.get_bottom_limit_cm()) ||
		   		(pilot_trans_thrusts.z > 0.0f && distance_control.get_top_limit_cm() != 0 && 
		   		distance_control.get_top_cm() > distance_control.get_top_limit_cm())) {
				is_affect_z = false;
		   	}
        }

        if (is_affect_z) {
	        // output pilot's throttle
	        motors.set_throttle_pilot(pilot_trans_thrusts.z);
	        attitude_control.set_throttle_out(motors.get_throttle_hover(), false, g.throttle_filt); // throttle should be zero
	        
	        // reset z targets to current values
	        pos_control.relax_alt_hold_controllers();
	        is_z_ctrl_relaxed = true;
	        engageStopZ = true;
	        lastVelocityZWasNegative = is_negative(inertial_nav.get_velocity_z());
	    } else { // hold z
	        
	        if (is_z_ctrl_relaxed) {
	            is_z_ctrl_relaxed = false;
	            pos_control.relax_alt_hold_controllers();
	        }

	        if (!is_ned_pilot && !depth_limit) {
	            motors.set_throttle_pilot(pilot_trans_thrusts.z); // here means throttle not affect z pos
	        } else {
	            motors.set_throttle_pilot(0.0f);
	        }

	        if (ap.at_bottom) {
	            pos_control.relax_alt_hold_controllers(); // clear velocity and position targets
	            pos_control.set_alt_target(inertial_nav.get_altitude() + 10.0f); // set target to 10 cm above bottom
	        }

	        // Detects a zero derivative
	        // When detected, move the altitude set point to the actual position
	        // This will avoid any problem related to joystick delays
	        // or smaller input signals
	        if(engageStopZ && (lastVelocityZWasNegative ^ is_negative(inertial_nav.get_velocity_z()))) {
	            engageStopZ = false;
	            pos_control.relax_alt_hold_controllers();
	        }

	        pos_control.update_z_controller();
	    }
	}

	if(is_ned_pilot) {
		if((distance_control.front_face_is_active() && 
		    (distance_control.get_front_cm() > 5 && distance_control.get_front_cm() < 200)) ||
		    (distance_control.back_face_is_active() && 
		    (distance_control.get_back_cm() < -5 && distance_control.get_back_cm() > -200))) {
		    int16_t distance;
		    if(distance_control.front_face_is_active()) {
		    	distance = distance_control.get_front_cm();
		    } else {
		    	distance = distance_control.get_back_cm();
		    }
		    
			if (fabsf(pilot_trans_thrusts.x) > 0.05f) {
		        // output pilot's throttle
		        motors.set_forward(pilot_trans_thrusts.x);
		        distance_control.relax_x_controller((float)distance);
		        last_pilot_x_input_ms = tnow;
		        is_x_ctrl_relaxed = true;
		    } else { // hold x
		        if (tnow - last_pilot_x_input_ms > 500) {
		        	if(is_x_ctrl_relaxed) {
			        	distance_control.relax_x_controller((float)distance);
			        	is_x_ctrl_relaxed = false;
		        	}
			    	distance_control.update_x_controller((float)distance);
		        } else {
					motors.set_forward(0);
		        }  
		    }
		} else if(distance_control.limit_enable()) {
			if(pilot_trans_thrusts.x >= 0.0f && distance_control.get_front_limit_cm() != 0 && 
				distance_control.get_front_cm() < distance_control.get_front_limit_cm()) {
				if(!is_equal(distance_control.get_target_x(), (float)distance_control.get_front_limit_cm())) {
		        	distance_control.relax_x_controller((float)distance_control.get_front_limit_cm());
	        	}
		    	distance_control.update_x_controller((float)distance_control.get_front_cm());
	        } else if(pilot_trans_thrusts.x <= -0.0f && distance_control.get_back_limit_cm() != 0 && 
		   		distance_control.get_back_cm() > distance_control.get_back_limit_cm()) {
				if(!is_equal(distance_control.get_target_x(), (float)distance_control.get_back_limit_cm())) {
		        	distance_control.relax_x_controller((float)distance_control.get_back_limit_cm());
	        	}
		    	distance_control.update_x_controller((float)distance_control.get_back_cm());
	        } else {
				motors.set_forward(pilot_trans_thrusts.x);
	        }
		} else {
			motors.set_forward(pilot_trans_thrusts.x);	
		}

	    if((distance_control.right_face_is_active() && 
	    	(distance_control.get_right_cm() > 5 && distance_control.get_right_cm() < 200)) ||
		    (distance_control.left_face_is_active() && 
		    (distance_control.get_left_cm() < -5 && distance_control.get_left_cm() > -200))) {
		    int16_t distance;
		    if(distance_control.right_face_is_active()) {
		    	distance = distance_control.get_right_cm();
		    } else {
		    	distance = distance_control.get_left_cm();
		    }
		    
			if (fabsf(pilot_trans_thrusts.y) > 0.05f) {
		        // output pilot's throttle
		        motors.set_lateral(pilot_trans_thrusts.y);
		        distance_control.relax_y_controller((float)distance);
		        last_pilot_y_input_ms = tnow;
		        is_y_ctrl_relaxed = true;
		    } else { // hold y
		        if (tnow - last_pilot_y_input_ms > 500) {
		        	if(is_y_ctrl_relaxed) {
			        	distance_control.relax_y_controller((float)distance);
			        	is_y_ctrl_relaxed = false;
		        	}
			    	distance_control.update_y_controller((float)distance);
		        } else {
					motors.set_lateral(0);
		        }  
		    }
		} else if(distance_control.limit_enable()) {
			if(pilot_trans_thrusts.y >= 0.0f && distance_control.get_right_limit_cm() != 0 && 
				distance_control.get_right_cm() < distance_control.get_right_limit_cm()) {
				//hal.shell->printf("right\r\n");
				if(!is_equal(distance_control.get_target_y(), (float)distance_control.get_right_limit_cm())) {
		        	distance_control.relax_y_controller((float)distance_control.get_right_limit_cm());
		        	hal.shell->printf("right relax\r\n");
	        	}
		    	distance_control.update_y_controller((float)distance_control.get_right_cm());
	        } else if(pilot_trans_thrusts.y <= -0.0f && distance_control.get_left_limit_cm() != 0 && 
		   		distance_control.get_left_cm() > distance_control.get_left_limit_cm()) {
		   		//hal.shell->printf("left\r\n");
				if(!is_equal(distance_control.get_target_y(), (float)distance_control.get_left_limit_cm())) {
		        	distance_control.relax_y_controller((float)distance_control.get_left_limit_cm());
		        	hal.shell->printf("left relax\r\n");
	        	}
		    	distance_control.update_y_controller((float)distance_control.get_left_cm());
	        } else {
				motors.set_lateral(pilot_trans_thrusts.y);
	        }
		} else {
			motors.set_lateral(pilot_trans_thrusts.y);	
		}
	} else {
		motors.set_forward(pilot_trans_thrusts.x);
		motors.set_lateral(pilot_trans_thrusts.y);
	}
}
