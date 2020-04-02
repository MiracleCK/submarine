#include "Sub.h"

// rtl_init - initialise guided controller
bool Sub::rtl_init(bool ignore_checks)
{
    if (!position_ok() && !ignore_checks) {
        return false;
    }

    // would not response to auto set home
    if (!ahrs.home_is_set() || !ahrs.home_is_locked()) {
        return false;
    }

    // initialise yaw
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
    // start in position control mode
    rtl_pos_control_start();

    is_waypoint_running = false;

    rtl_set_destination(ahrs.get_home());

    return true;
}

// initialise guided mode's position controller
void Sub::rtl_pos_control_start()
{
    // set to position control mode
    guided_mode = Guided_WP;

    // initialise waypoint and spline controller
    wp_nav.wp_and_spline_init();

    // initialise wpnav to stopping point at current altitude
    // To-Do: set to current location if disarmed?
    // To-Do: set to stopping point altitude?
    Vector3f stopping_point;
    stopping_point.z = inertial_nav.get_altitude();
    wp_nav.get_wp_stopping_point_xy(stopping_point);

    // no need to check return status because terrain data is not used
    wp_nav.set_wp_destination(stopping_point, false);
    printf("stopping_x stopping_y stopping_z = %f %f %f \r\n", stopping_point.x, stopping_point.y, stopping_point.z);

    // initialise yaw
    set_auto_yaw_mode(get_default_auto_yaw_mode(false));
}

void Sub::rtl_run()
{
    if (wp_nav.reached_wp_destination()) {
        if (is_waypoint_running == true) {
            gcs().send_mission_item_reached_message(1);
            is_waypoint_running = false;
            is_mode_auto_switch_enabled = true;

            // switch to manual then auto select a mode
            sub.set_mode(MANUAL, ModeReason::GUIDED_DONE);
        }
    }

    // if motors not enabled set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        // Sub vehicles do not stabilize roll/pitch/yaw when disarmed
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        return;
    }

    // process pilot's yaw input
    float target_yaw_rate = 0;
    if (!failsafe.pilot_input) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            set_auto_yaw_mode(AUTO_YAW_HOLD);
        }
    }

    // set motors to full range
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    failsafe_terrain_set_status(wp_nav.update_wpnav());

    float lateral_out, forward_out;
    translate_wpnav_rp(lateral_out, forward_out);

    // Send to forward/lateral outputs
    motors.set_lateral(lateral_out);
    motors.set_forward(forward_out);

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control.update_z_controller();

    // call attitude controller
    if (auto_yaw_mode == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_yaw_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from auto_heading()
        attitude_control.input_euler_angle_roll_pitch_yaw(channel_roll->get_control_in(), channel_pitch->get_control_in(), get_auto_heading(), true);
    }
}

// sets rtl mode's target from a Location object
// returns false if destination could not be set (probably caused by missing terrain data)
// or if the fence is enabled and rtl waypoint is outside the fence
bool Sub::rtl_set_destination(const Location& dest_loc)
{
    Location current_pos;
    Location temp_loc;

    temp_loc = dest_loc;
    ahrs.get_location(current_pos);
    temp_loc.alt = current_pos.alt;

    // Location ekf_position;

    // ensure we are in position control mode
    if (guided_mode != Guided_WP) {
        guided_pos_control_start();
    }

#if AC_FENCE == ENABLED
    // reject destination outside the fence.
    // Note: there is a danger that a target specified as a terrain altitude might not be checked if the conversion to alt-above-home fails
    // if (!fence.check_destination_within_fence(dest_loc)) {
    if (!fence.check_destination_within_fence(temp_loc)) {
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::DEST_OUTSIDE_FENCE);
        // failure is propagated to GCS with NAK
        return false;
    }
#endif
    
    // ahrs.get_location(ekf_position);
    // dest_loc.alt = ekf_position.alt;

    // if (!wp_nav.set_wp_destination(dest_loc)) {
    if (!wp_nav.set_wp_destination(temp_loc)) {
        // failure to set destination can only be because of missing terrain data
        AP::logger().Write_Error(LogErrorSubsystem::NAVIGATION, LogErrorCode::FAILED_TO_SET_DESTINATION);
        // failure is propagated to GCS with NAK
        return false;
    }

    is_waypoint_running = true;

    // log target
    Log_Write_GuidedTarget(guided_mode, Vector3f(dest_loc.lat, dest_loc.lng, dest_loc.alt),Vector3f());
    return true;
}