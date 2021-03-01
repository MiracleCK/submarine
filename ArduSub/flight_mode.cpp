#include "Sub.h"

// change flight mode and perform any necessary initialisation
// returns true if mode was successfully set
bool Sub::set_mode(control_mode_t mode, ModeReason reason)
{
    // if set_mode called by MAVLink or joystick
    // we do not do smart mode auto switch again
    if (reason == ModeReason::GCS_COMMAND || reason == ModeReason::RC_COMMAND) {
        is_mode_auto_switch_enabled = false;
    }

    // boolean to record if flight mode could be set
    bool success = false;

    // return immediately if we are already in the desired mode
    if (mode == control_mode) {
        prev_control_mode = control_mode;
        prev_control_mode_reason = control_mode_reason;

        control_mode_reason = reason;
        return true;
    }

    switch (mode) {
    case ACRO:
        success = acro_init();
        break;

    case STABILIZE:
        success = stabilize_init();
        break;

    case ALT_HOLD:
        success = althold_init();
        break;

    case AUTO:
        success = auto_init();
        break;

    case CIRCLE:
        success = circle_init();
        break;

    case GUIDED:
        success = guided_init();
        break;

    case SURFACE:
        success = surface_init();
        break;

#if POSHOLD_ENABLED == ENABLED
    case POSHOLD:
        success = poshold_init();
        break;
#endif

    case MANUAL:
        success = manual_init();
        break;

    case MOTOR_DETECT:
        success = motordetect_init();
        break;

    default:
        success = false;
        break;
    }

    // update flight mode
    if (success) {
        // perform any cleanup required by previous flight mode
        exit_mode(control_mode, mode);

        prev_control_mode = control_mode;
        prev_control_mode_reason = control_mode_reason;

        control_mode = mode;
        control_mode_reason = reason;
        logger.Write_Mode(control_mode, control_mode_reason);
        gcs().send_message(MSG_HEARTBEAT);

        // update notify object
        notify_flight_mode(control_mode);

#if CAMERA == ENABLED
        camera.set_is_auto_mode(control_mode == AUTO);
#endif

#if AC_FENCE == ENABLED
        // pilot requested flight mode change during a fence breach indicates pilot is attempting to manually recover
        // this flight mode change could be automatic (i.e. fence, battery, GPS or GCS failsafe)
        // but it should be harmless to disable the fence temporarily in these situations as well
        fence.manual_recovery_start();
#endif
    } else {
        // Log error that we failed to enter desired flight mode
        AP::logger().Write_Error(LogErrorSubsystem::FLIGHT_MODE, LogErrorCode(mode));
    }

    // return success or failure
    return success;
}

bool Sub::set_mode(const uint8_t new_mode, const ModeReason reason)
{
    static_assert(sizeof(control_mode_t) == sizeof(new_mode), "The new mode can't be mapped to the vehicles mode number");
    return sub.set_mode((control_mode_t)new_mode, reason);
}

// update_flight_mode - calls the appropriate attitude controllers based on flight mode
// called at 100hz or more
void Sub::update_flight_mode()
{
    if (motors.armed()) {
		distance_bf[DISTANCE_FRONT] = 0;//rangefinder.distance_cm_filtered_orient(ROTATION_NONE);
        distance_bf[DISTANCE_BACK] = 0;//rangefinder.distance_cm_filtered_orient(ROTATION_PITCH_180);
        distance_bf[DISTANCE_LEFT] = 0;//rangefinder.distance_cm_filtered_orient(ROTATION_YAW_270);
        distance_bf[DISTANCE_RIGHT] = 0;//rangefinder.distance_cm_filtered_orient(ROTATION_YAW_90);
        distance_bf[DISTANCE_TOP] = 0;
        distance_bf[DISTANCE_BOTTOM] = rangefinder.distance_cm_filtered_orient(ROTATION_PITCH_270);

		Matrix3f m;
		m.from_euler(ahrs.roll, ahrs.pitch, 0);

		Vector3f dist[DISTANCE_NUM] = {{(float)distance_bf[DISTANCE_FRONT], 0, 0},
							{(float)-distance_bf[DISTANCE_BACK], 0, 0},
							{0, (float)-distance_bf[DISTANCE_LEFT], 0},
							{0, (float)distance_bf[DISTANCE_RIGHT], 0},
							{0, 0, (float)-distance_bf[DISTANCE_TOP]},
							{0, 0, (float)distance_bf[DISTANCE_BOTTOM]}};
		for(int i=0; i<DISTANCE_NUM; i++) {
			dist[i] = m * dist[i];
			distance_ned[i] = 0;
		}

		for(int i=0; i<DISTANCE_NUM; i++) {
			if(dist[i].x >= 0) {
				if(dist[i].x > distance_ned[DISTANCE_FRONT])
					distance_ned[DISTANCE_FRONT] = dist[i].x;
			} else {
				if(dist[i].x < distance_ned[DISTANCE_BACK])
					distance_ned[DISTANCE_BACK] = dist[i].x;
			}

			if(dist[i].y >= 0) {
				if(dist[i].y > distance_ned[DISTANCE_RIGHT])
					distance_ned[DISTANCE_RIGHT] = dist[i].y;
			} else {
				if(dist[i].y < distance_ned[DISTANCE_LEFT])
					distance_ned[DISTANCE_LEFT] = dist[i].y;
			}

			if(dist[i].z >= 0) {
				if(dist[i].z > distance_ned[DISTANCE_BOTTOM])
					distance_ned[DISTANCE_BOTTOM] = dist[i].z;
			} else {
				if(dist[i].z < distance_ned[DISTANCE_TOP])
					distance_ned[DISTANCE_TOP] = dist[i].z;
			}
		}

		if(0) {
			static uint32_t _startup_ms = 0;

			if(_startup_ms == 0) {
				_startup_ms = AP_HAL::millis();
			}

			if(AP_HAL::millis() - _startup_ms > 1000) {
				_startup_ms = AP_HAL::millis();

				//for(int i=0; i<DISTANCE_NUM; i++) {
				//	hal.shell->printf("[%.2f %.2f %.2f]\r\n", 
				//				dist[i].x,
				//				dist[i].y, 
				//				dist[i].z);
				//}
				
				hal.shell->printf("\r\ndistance_bf: %d %d %d %d %d %d\r\n", 
							distance_bf[DISTANCE_FRONT],
							distance_bf[DISTANCE_BACK], 
							distance_bf[DISTANCE_LEFT], 
							distance_bf[DISTANCE_RIGHT],
							distance_bf[DISTANCE_TOP],
							distance_bf[DISTANCE_BOTTOM]);

				hal.shell->printf("distance_ned: %d %d %d %d %d %d\r\n\r\n", 
							distance_ned[DISTANCE_FRONT],
							distance_ned[DISTANCE_BACK], 
							distance_ned[DISTANCE_LEFT], 
							distance_ned[DISTANCE_RIGHT],
							distance_ned[DISTANCE_TOP],
							distance_ned[DISTANCE_BOTTOM]);
			}
		}

		pilot_trans_thrusts.x = channel_forward->slew_norm_input_bidirectional();
        pilot_trans_thrusts.y = channel_lateral->slew_norm_input_bidirectional();
        pilot_trans_thrusts.z = channel_throttle->slew_norm_input_bidirectional();
        
        //if(g.distance_enable && is_ned_pilot) {
        if(0) {
			Vector3f dis_error;
			
	        if(pilot_trans_thrusts.x > 0.05f) {
	    		if(ahrs.pitch_sensor > -4500)
					dis_error.x = distance_ned[DISTANCE_FRONT] - 50;//g.distance_limit;
				else
					dis_error.x = g.distance_p/2;
	    	} else if(pilot_trans_thrusts.x < -0.05f) {
	    		if(ahrs.pitch_sensor < 4500)
					dis_error.x = abs(distance_ned[DISTANCE_BACK]) - 50;//g.distance_limit;
				else
					dis_error.x = g.distance_p;
	    	} else {
				dis_error.x = 0;
				pilot_trans_thrusts.x = 0;
	    	}
	    	//if(is_zero(pilot_trans_thrusts.x) || dis_error.x < 5)
	    	//	dis_error.x = 0;
	    	//if(dis_error.x < 0)
	    	//	dis_error.x *= fabsf(dis_error.x);
	    	pilot_trans_thrusts.x *= constrain_float((float)dis_error.x/g.distance_p, -1.0f, 1.0f);

	    	if(pilot_trans_thrusts.y > 0.05f) {
	    		if(ahrs.roll_sensor < 4500)
					dis_error.y = distance_ned[DISTANCE_RIGHT] - 50;//g.distance_limit;
				else
					dis_error.y = g.distance_p;
	    	} else if(ahrs.roll_sensor > -4500) {
				if(abs(distance_ned[DISTANCE_LEFT]) > 10)
					dis_error.y = abs(distance_ned[DISTANCE_LEFT]) - 50;//g.distance_limit;
				else
					dis_error.y = g.distance_p;
	    	} else {
				dis_error.y = 0;
				pilot_trans_thrusts.y = 0;
	    	}
	    	//if(is_zero(pilot_trans_thrusts.y) || dis_error.y < 5)
	    	//	dis_error.y = 0;
	    	//if(dis_error.y < 0)
	    	//	dis_error.y *= fabsf(dis_error.y);
	    	pilot_trans_thrusts.y *= constrain_float((float)dis_error.y/g.distance_p, -1.0f, 1.0f);

			if(pilot_trans_thrusts.z < -0.05f) {
				dis_error.z = distance_ned[DISTANCE_BOTTOM] - 15;//g.distance_limit;
			} else if(pilot_trans_thrusts.z > 0.05f) {
				if(abs(distance_ned[DISTANCE_TOP]) > 10)
					dis_error.z = abs(distance_ned[DISTANCE_TOP]) - 15;//g.distance_limit;
				else
					dis_error.z = g.distance_p;
			} else {
				dis_error.z = 0;
				pilot_trans_thrusts.z = 0;
			}
			//if(is_zero(pilot_trans_thrusts.z) || dis_error.z < 5)
			//	dis_error.z = 0;
			pilot_trans_thrusts.z *= constrain_float((float)dis_error.z/g.distance_p, 0.0f, 1.0f);

	    	if(0) {
				static uint32_t _startup_ms = 0;

				if(_startup_ms == 0) {
					_startup_ms = AP_HAL::millis();
				}

				if(AP_HAL::millis() - _startup_ms > 1000) {
					_startup_ms = AP_HAL::millis();

					hal.shell->printf("dis_error [%.4f %.4f %.4f]\r\n",
							dis_error.x, 
							dis_error.y, 
							dis_error.z);

					hal.shell->printf("pilot_trans_thrusts [%.4f %.4f %.4f]\r\n",
							pilot_trans_thrusts.x, 
							pilot_trans_thrusts.y,
							pilot_trans_thrusts.z); 
				}
			}
		}

        is_affect_z = is_affect_z_pos(is_ned_pilot, pilot_trans_thrusts.x, pilot_trans_thrusts.y, pilot_trans_thrusts.z);
        thrust_decomposition_select(is_ned_pilot, control_mode, is_affect_z);
    } else {
        pilot_trans_thrusts(0, 0, 0);
    }

    switch (control_mode) {
    case ACRO:
        acro_run();
        break;

    case STABILIZE:
        stabilize_run();
        break;

    case ALT_HOLD:
        althold_run();
        break;

    case AUTO:
        auto_run();
        break;

    case CIRCLE:
        circle_run();
        break;

    case GUIDED:
        guided_run();
        break;

    case SURFACE:
        surface_run();
        break;

#if POSHOLD_ENABLED == ENABLED
    case POSHOLD:
        poshold_run();
        break;
#endif

    case MANUAL:
        manual_run();
        break;

    case MOTOR_DETECT:
        motordetect_run();
        break;

    default:
        break;
    }
}

// exit_mode - high level call to organise cleanup as a flight mode is exited
void Sub::exit_mode(control_mode_t old_control_mode, control_mode_t new_control_mode)
{
    // stop mission when we leave auto mode
    if (old_control_mode == AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();
        }
#if MOUNT == ENABLED
        camera_mount.set_mode_to_default();
#endif  // MOUNT == ENABLED
    }
}

// returns true or false whether mode requires GPS
bool Sub::mode_requires_GPS(control_mode_t mode)
{
    switch (mode) {
    case AUTO:
    case GUIDED:
    case CIRCLE:
    case POSHOLD:
        return true;
    default:
        return false;
    }

    return false;
}

// mode_has_manual_throttle - returns true if the flight mode has a manual throttle (i.e. pilot directly controls throttle)
bool Sub::mode_has_manual_throttle(control_mode_t mode)
{
    switch (mode) {
    case ACRO:
    case STABILIZE:
    case MANUAL:
        return true;
    default:
        return false;
    }

    return false;
}

// mode_allows_arming - returns true if vehicle can be armed in the specified mode
//  arming_from_gcs should be set to true if the arming request comes from the ground station
bool Sub::mode_allows_arming(control_mode_t mode, bool arming_from_gcs)
{
    return (mode_has_manual_throttle(mode)
        || mode == ALT_HOLD
        || mode == POSHOLD
        || (arming_from_gcs&& mode == GUIDED)
    );
}

// notify_flight_mode - sets notify object based on flight mode.  Only used for OreoLED notify device
void Sub::notify_flight_mode(control_mode_t mode)
{
    switch (mode) {
    case AUTO:
    case GUIDED:
    case CIRCLE:
    case SURFACE:
        // autopilot modes
        AP_Notify::flags.autopilot_mode = true;
        break;
    default:
        // all other are manual flight modes
        AP_Notify::flags.autopilot_mode = false;
        break;
    }
}

// auto switch mode according to sensor healthy
// and stop auto switch when mode set by GCS or RC
//
// true: auto switched
// false: not do auto switch
bool Sub::smart_mode_auto_switch() {
    if (!is_mode_auto_switch_enabled) {
        return false;
    }

    ModeReason reason = ModeReason::STARTUP;
    bool is_success = false;

    if (!is_startup_mode_auto_switch) {
        reason = ModeReason::BAD_DEPTH;
    }

    switch (control_mode) {
        case MANUAL: {
            if (!sub.set_mode(ALT_HOLD, reason)) {
                is_success = sub.set_mode(STABILIZE, reason);
            } else {
                is_success = true;
            }
        } break;

        case STABILIZE: {
            is_success = sub.set_mode(ALT_HOLD, reason);
        } break;

        case ALT_HOLD: {
            if(!control_check_barometer()) {
                is_success = sub.set_mode(STABILIZE, reason);
            }
        } break;

        default:
            break;
    }

    if (is_success) {
        is_startup_mode_auto_switch = false;
    }

    return is_success;
}
