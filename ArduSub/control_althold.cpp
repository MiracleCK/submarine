#include "Sub.h"


bool use_angle_rate = false;

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

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void Sub::althold_run()
{
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
    static bool engageStopZ = true;
    // Get last user velocity direction to check for zero derivative points
    static bool lastVelocityZWasNegative = false;
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

void Sub::get_alt_hold_pilot_desired_rate_lean_angles(float roll_rate_in, float pitch_rate_in,
    float &roll_out, float &pitch_out)
{
    float roll_control_in, pitch_control_in;
    float angle_max;

    // sanity check angle max parameter
    angle_max = constrain_int16(aparm.angle_max,1000,8000);

    // map angle to control in
    float scaler = (float)ROLL_PITCH_INPUT_MAX / angle_max;

    roll_control_in = (last_pilot_roll + roll_rate_in * G_Dt) * scaler;
    pitch_control_in = (last_pilot_pitch + pitch_rate_in * G_Dt) * scaler;

    get_pilot_desired_lean_angles(roll_control_in, pitch_control_in, roll_out, pitch_out, attitude_control.get_althold_lean_angle_max());
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


void Sub::althold_run_rate()
{
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
        last_pilot_pitch = ahrs.pitch_sensor;
        last_pilot_roll = ahrs.roll_sensor;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // get pilot desired lean angles
    float target_roll, target_pitch;
    float target_roll_rate, target_pitch_rate, target_yaw_rate;

    get_alt_hold_pilot_desired_angle_rates(
        channel_roll->get_control_in(), channel_pitch->get_control_in(), channel_yaw->get_control_in(), 
        target_roll_rate, target_pitch_rate, target_yaw_rate);
    
    get_alt_hold_pilot_desired_rate_lean_angles(target_roll_rate, target_pitch_rate, target_roll, target_pitch);

    if (is_request_reset_rp) {
        target_pitch = 0;
        target_roll = 0;

        is_request_reset_rp = false;
    }

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

    last_pilot_pitch = target_pitch;
    last_pilot_roll = target_roll;

    // Hold actual position until zero derivative is detected
    static bool engageStopZ = true;
    // Get last user velocity direction to check for zero derivative points
    static bool lastVelocityZWasNegative = false;
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
