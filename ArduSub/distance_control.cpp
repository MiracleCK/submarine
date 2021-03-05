#include <AP_HAL/AP_HAL.h>
#include "distance_control.h"
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

 // default gains for Sub
 #define DISCONTROL_POS_Z_P                    3.0f    // vertical position controller P gain default
 #define DISCONTROL_VEL_Z_P                    8.0f    // vertical velocity controller P gain default
 #define DISCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 #define DISCONTROL_ACC_Z_I                    0.1f    // vertical acceleration controller I gain default
 #define DISCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 #define DISCONTROL_ACC_Z_IMAX                 100     // vertical acceleration controller IMAX gain default
 #define DISCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 #define DISCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 #define DISCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 #define DISCONTROL_VEL_XY_P                   1.0f    // horizontal velocity controller P gain default
 #define DISCONTROL_VEL_XY_I                   0.5f    // horizontal velocity controller I gain default
 #define DISCONTROL_VEL_XY_D                   0.0f    // horizontal velocity controller D gain default
 #define DISCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 #define DISCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 #define DISCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D

// vibration compensation gains
#define DISCONTROL_VIBE_COMP_P_GAIN 0.250f
#define DISCONTROL_VIBE_COMP_I_GAIN 0.125f

const AP_Param::GroupInfo DistanceControl::var_info[] = {
   	// @Param: _POSZ_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_z, "_POSZ_", 1, DistanceControl, AC_P),

    // @Param: _VELZ_P
    // @DisplayName: Velocity (vertical) controller P gain
    // @Description: Velocity (vertical) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_vel_z, "_VELZ_", 2, DistanceControl, AC_P),

    // @Param: _ACCZ_P
    // @DisplayName: Acceleration (vertical) controller P gain
    // @Description: Acceleration (vertical) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @Increment: 0.05
    // @User: Standard

    // @Param: _ACCZ_I
    // @DisplayName: Acceleration (vertical) controller I gain
    // @Description: Acceleration (vertical) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: _ACCZ_IMAX
    // @DisplayName: Acceleration (vertical) controller I gain maximum
    // @Description: Acceleration (vertical) controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: d%
    // @User: Standard

    // @Param: _ACCZ_D
    // @DisplayName: Acceleration (vertical) controller D gain
    // @Description: Acceleration (vertical) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACCZ_FILT
    // @DisplayName: Acceleration (vertical) controller filter
    // @Description: Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_accel_z, "_ACCZ_", 3, DistanceControl, AC_PID),

    // @Param: _POSZ_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_x, "_POSX_", 4, DistanceControl, AC_P),

    // @Param: _VELZ_P
    // @DisplayName: Velocity (vertical) controller P gain
    // @Description: Velocity (vertical) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_vel_x, "_VELX_", 5, DistanceControl, AC_P),

    // @Param: _ACCZ_P
    // @DisplayName: Acceleration (vertical) controller P gain
    // @Description: Acceleration (vertical) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @Increment: 0.05
    // @User: Standard

    // @Param: _ACCZ_I
    // @DisplayName: Acceleration (vertical) controller I gain
    // @Description: Acceleration (vertical) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: _ACCZ_IMAX
    // @DisplayName: Acceleration (vertical) controller I gain maximum
    // @Description: Acceleration (vertical) controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: d%
    // @User: Standard

    // @Param: _ACCZ_D
    // @DisplayName: Acceleration (vertical) controller D gain
    // @Description: Acceleration (vertical) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACCZ_FILT
    // @DisplayName: Acceleration (vertical) controller filter
    // @Description: Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_accel_x, "_ACCX_", 6, DistanceControl, AC_PID),
    
    // @Param: _POSZ_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_y, "_POSY_", 7, DistanceControl, AC_P),

    // @Param: _VELZ_P
    // @DisplayName: Velocity (vertical) controller P gain
    // @Description: Velocity (vertical) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_vel_y, "_VELY_", 8, DistanceControl, AC_P),

    // @Param: _ACCZ_P
    // @DisplayName: Acceleration (vertical) controller P gain
    // @Description: Acceleration (vertical) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @Increment: 0.05
    // @User: Standard

    // @Param: _ACCZ_I
    // @DisplayName: Acceleration (vertical) controller I gain
    // @Description: Acceleration (vertical) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: _ACCZ_IMAX
    // @DisplayName: Acceleration (vertical) controller I gain maximum
    // @Description: Acceleration (vertical) controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: d%
    // @User: Standard

    // @Param: _ACCZ_D
    // @DisplayName: Acceleration (vertical) controller D gain
    // @Description: Acceleration (vertical) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACCZ_FILT
    // @DisplayName: Acceleration (vertical) controller filter
    // @Description: Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_accel_y, "_ACCY_", 9, DistanceControl, AC_PID),

    // @Param: HOLD_ENABLE
    // @DisplayName: Keep a distance and avoid collision
    // @Description: Keep a distance and avoid collision
    // @Values: 1:enable,0:disable
    // @User: Advanced
    AP_GROUPINFO("HOLD_ENABLE",  14, DistanceControl, _hold_enable, 0),

    // @Param: FIX_ENABLE
    // @DisplayName: Fixed distance
    // @Description: Fixed distance
    // @Values: 1:enable,0:disable
    // @User: Advanced
    AP_GROUPINFO("FIX_ENABLE",  15, DistanceControl, _fix_enable, 0),

    AP_GROUPINFO("OFT_FRONT",  16, DistanceControl, _offset_front, 0),
    AP_GROUPINFO("OFT_BACK",  17, DistanceControl, _offset_back, 0),
    AP_GROUPINFO("OFT_LEFT",  18, DistanceControl, _offset_left, 0),
    AP_GROUPINFO("OFT_RIGHT",  19, DistanceControl, _offset_right, 0),

    AP_GROUPINFO("FRONT_CM",  20, DistanceControl, _front_cm, 0),
    AP_GROUPINFO("BACK_CM",  21, DistanceControl, _back_cm, 0),
    AP_GROUPINFO("LEFT_CM",  22, DistanceControl, _left_cm, 0),
    AP_GROUPINFO("RIGHT_CM",  23, DistanceControl, _right_cm, 0),
	AP_GROUPINFO("BOTTOM_CM",  24, DistanceControl, _bottom_cm, 0),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
DistanceControl::DistanceControl(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                             AP_Motors& motors, AC_AttitudeControl& attitude_control,
                             const RangeFinder& rangefinder) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control),
    _rangefinder(rangefinder),
    _p_pos_x(DISCONTROL_POS_Z_P),
    _p_vel_x(DISCONTROL_VEL_Z_P),
    _pid_accel_x(DISCONTROL_ACC_Z_P, DISCONTROL_ACC_Z_I, DISCONTROL_ACC_Z_D, 0.0f, DISCONTROL_ACC_Z_IMAX, 0.0f, DISCONTROL_ACC_Z_FILT_HZ, 0.0f, DISCONTROL_ACC_Z_DT),
	_p_pos_y(DISCONTROL_POS_Z_P),
    _p_vel_y(DISCONTROL_VEL_Z_P),
    _pid_accel_y(DISCONTROL_ACC_Z_P, DISCONTROL_ACC_Z_I, DISCONTROL_ACC_Z_D, 0.0f, DISCONTROL_ACC_Z_IMAX, 0.0f, DISCONTROL_ACC_Z_FILT_HZ, 0.0f, DISCONTROL_ACC_Z_DT),
    _p_pos_z(DISCONTROL_POS_Z_P),
    _p_vel_z(DISCONTROL_VEL_Z_P),
    _pid_accel_z(DISCONTROL_ACC_Z_P, DISCONTROL_ACC_Z_I, DISCONTROL_ACC_Z_D, 0.0f, DISCONTROL_ACC_Z_IMAX, 0.0f, DISCONTROL_ACC_Z_FILT_HZ, 0.0f, DISCONTROL_ACC_Z_DT),
    _dt(DISCONTROL_DT_400HZ),
    _accel_cms(DISCONTROL_ACCEL_X, DISCONTROL_ACCEL_Y, DISCONTROL_ACCEL_Z),
    _vel_x_error_filter(DISCONTROL_VEL_ERROR_CUTOFF_FREQ),
    _vel_y_error_filter(DISCONTROL_VEL_ERROR_CUTOFF_FREQ),
    _vel_z_error_filter(DISCONTROL_VEL_ERROR_CUTOFF_FREQ),
    _out_x_filter(DISCONTROL_THROTTLE_CUTOFF_FREQ),
    _out_y_filter(DISCONTROL_THROTTLE_CUTOFF_FREQ)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _flags.reset_rate_to_accel_x = true;
    _flags.freeze_ff_x = true;
    _flags.use_desvel_ff_x = true;
    
    _flags.reset_rate_to_accel_y = true;
    _flags.freeze_ff_y = true;
    _flags.use_desvel_ff_y = true;
    
    _flags.reset_rate_to_accel_z = true;
    _flags.freeze_ff_z = true;
    _flags.use_desvel_ff_z = true;
}

void DistanceControl::update_distance(void)
{
	distance_bf[DISTANCE_FRONT] = _rangefinder.distance_cm_filtered_orient(ROTATION_NONE);
    distance_bf[DISTANCE_BACK] = _rangefinder.distance_cm_filtered_orient(ROTATION_PITCH_180);
    distance_bf[DISTANCE_LEFT] = _rangefinder.distance_cm_filtered_orient(ROTATION_YAW_270);
    distance_bf[DISTANCE_RIGHT] = _rangefinder.distance_cm_filtered_orient(ROTATION_YAW_90);
    distance_bf[DISTANCE_TOP] = 0;
    distance_bf[DISTANCE_BOTTOM] = _rangefinder.distance_cm_filtered_orient(ROTATION_PITCH_270);

	Matrix3f m;
	m.from_euler(_ahrs.roll, _ahrs.pitch, 0);

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

	distance_ned[DISTANCE_FRONT] -= _front_cm;
	distance_ned[DISTANCE_BACK] += _back_cm;
	distance_ned[DISTANCE_RIGHT] -= _right_cm;
	distance_ned[DISTANCE_LEFT] += _left_cm;
	distance_ned[DISTANCE_BOTTOM] -= _bottom_cm;

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
}

void DistanceControl::relax_z_controllers(float distance)
{
    _pos_target.z = distance;
    _vel_desired.z = 0.0f;
    _flags.use_desvel_ff_z = false;
    _vel_target.z = _inav.get_velocity_z();
    _vel_last.z = _inav.get_velocity_z();
    _accel_desired.z = 0.0f;
    _flags.reset_rate_to_accel_z = true;
    _accel_target.z = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
    _pid_accel_z.reset_filter();
}

Vector3f vel_cm, accl_cm;
void DistanceControl::update_z_controller(float distance)
{
    float curr_alt = distance;
    static uint8_t print_flag = 0;

    if(1) {
		static uint32_t _startup_ms = 0;

		if(_startup_ms == 0) {
			_startup_ms = AP_HAL::millis();
		}

		if(AP_HAL::millis() - _startup_ms > 1000) {
			_startup_ms = AP_HAL::millis();
			
			print_flag = 1;
		}
	}

    // calculate altitude error
    _pos_error.z = _pos_target.z - curr_alt;

    if(print_flag) {
		hal.shell->printf("pt[%.4f] pc[%.4f] pe[%.4f]\r\n", 
					_pos_target.z,
					curr_alt,
					_pos_error.z);
	}

    // calculate _vel_target.z using from _pos_error.z using sqrt controller
    _vel_target.z = AC_AttitudeControl::sqrt_controller(_pos_error.z, _p_pos_z.kP(), _accel_cms.z, _dt);

    // add feed forward component
    if (_flags.use_desvel_ff_z) {
        _vel_target.z += _vel_desired.z;
    }

    // the following section calculates acceleration required to achieve the velocity target

    const Vector3f& curr_vel = _inav.get_velocity();
    vel_cm.z = curr_vel.z;

    // TODO: remove velocity derivative calculation
    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_z) {
        _vel_last.z = _vel_target.z;
    }

    // feed forward desired acceleration calculation
    if (_dt > 0.0f) {
        if (!_flags.freeze_ff_z) {
            _accel_desired.z = (_vel_target.z - _vel_last.z) / _dt;
        } else {
            // stop the feed forward being calculated during a known discontinuity
            _flags.freeze_ff_z = false;
        }
    } else {
        _accel_desired.z = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.z = _vel_target.z;

    // reset velocity error and filter if this controller has just been engaged
    if (_flags.reset_rate_to_accel_z) {
        // Reset Filter
        _vel_error.z = 0;
        _vel_z_error_filter.reset(0);
        _flags.reset_rate_to_accel_z = false;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        _vel_error.z = _vel_z_error_filter.apply(_vel_target.z - curr_vel.z, _dt);
    }

	if(print_flag) {
		hal.shell->printf("vt[%.4f] vc[%.4f] ve[%.4f]\r\n", 
					_vel_target.z,
					curr_vel.z,
					_vel_error.z);
	}
	
    _accel_target.z = _p_vel_z.get_p(_vel_error.z);

    _accel_target.z += _accel_desired.z;


    // the following section calculates a desired throttle needed to achieve the acceleration target
    float z_accel_meas;         // actual acceleration

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
	accl_cm.z = z_accel_meas;
	
    // ensure imax is always large enough to overpower hover throttle
    if (_motors.get_throttle_hover() * 1000.0f > _pid_accel_z.imax()) {
        _pid_accel_z.imax(_motors.get_throttle_hover() * 1000.0f);
    }

    float pid_out, thr_out;
    if (_vibe_comp_enabled) {
        _flags.freeze_ff_z = true;
        _accel_desired.z = 0.0f;
        const float thr_per_accelz_cmss = _motors.get_throttle_hover() / (GRAVITY_MSS * 100.0f);
        // during vibration compensation use feed forward with manually calculated gain
        // ToDo: clear pid_info P, I and D terms for logging
        if (!(_motors.limit.throttle_lower || _motors.limit.throttle_upper) || ((is_positive(_pid_accel_z.get_i()) && is_negative(_vel_error.z)) || (is_negative(_pid_accel_z.get_i()) && is_positive(_vel_error.z)))) {
            _pid_accel_z.set_integrator(_pid_accel_z.get_i() + _dt * thr_per_accelz_cmss * 1000.0f * _vel_error.z * _p_vel_z.kP() * DISCONTROL_VIBE_COMP_I_GAIN);
        }
        pid_out = DISCONTROL_VIBE_COMP_P_GAIN * thr_per_accelz_cmss * _accel_target.z + _pid_accel_z.get_i() * 0.001f;
    } else {
        pid_out = _pid_accel_z.update_all(_accel_target.z, z_accel_meas, (_motors.limit.throttle_lower || _motors.limit.throttle_upper)) * 0.001f;
    }

    thr_out = pid_out + _motors.get_throttle_hover();

    if(print_flag) {
		hal.shell->printf("at[%.4f] ac[%.4f] ae[%.4f] pout[%.4f] hover[%.4f] out[%.4f]\r\n", 
					_accel_target.z,
					z_accel_meas,
					_accel_target.z - z_accel_meas,
					pid_out,
					_motors.get_throttle_hover(),
					thr_out);
	}

    // send throttle to attitude controller with angle boost
    _attitude_control.set_throttle_out(thr_out, true, DISCONTROL_THROTTLE_CUTOFF_FREQ);
    
    print_flag = 0;
}


void DistanceControl::relax_x_controllers(float distance)
{
    _pos_target.x = distance;
    _vel_desired.x = 0.0f;
    _flags.use_desvel_ff_x = false;
    _vel_target.x = _inav.get_velocity().x;
    _vel_last.x = _vel_target.x;
    _accel_desired.x = 0.0f;
    _flags.reset_rate_to_accel_x = true;
    _accel_target.x = _ahrs.get_accel_ef_blended().x * 100.0f;
    _pid_accel_x.reset_filter();
}

void DistanceControl::update_x_controller(float distance)
{
    float curr_dis = distance;
    static uint8_t print_flag = 0;

    if(1) {
		static uint32_t _startup_ms = 0;

		if(_startup_ms == 0) {
			_startup_ms = AP_HAL::millis();
		}

		if(AP_HAL::millis() - _startup_ms > 1000) {
			_startup_ms = AP_HAL::millis();
			
			print_flag = 1;
		}
	}

    // calculate altitude error
    _pos_error.x = _pos_target.x - curr_dis;

    if(print_flag) {
		hal.shell->printf("pt[%.4f] pc[%.4f] pe[%.4f]\r\n", 
					_pos_target.x,
					curr_dis,
					_pos_error.x);
	}

    // calculate _vel_target.x using from _pos_error.x using sqrt controller
    _vel_target.x = AC_AttitudeControl::sqrt_controller(_pos_error.x, _p_pos_x.kP(), _accel_cms.x, _dt);

    // add feed forward component
    if (_flags.use_desvel_ff_x) {
        _vel_target.x += _vel_desired.x;
    }

    // the following section calculates acceleration required to achieve the velocity target

    const Vector3f& curr_vel = _inav.get_velocity();

    // TODO: remove velocity derivative calculation
    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_x) {
        _vel_last.x = _vel_target.x;
    }

    // feed forward desired acceleration calculation
    if (_dt > 0.0f) {
        if (!_flags.freeze_ff_x) {
            _accel_desired.x = (_vel_target.x - _vel_last.x) / _dt;
        } else {
            // stop the feed forward being calculated during a known discontinuity
            _flags.freeze_ff_x = false;
        }
    } else {
        _accel_desired.x = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.x = _vel_target.x;

    // reset velocity error and filter if this controller has just been engaged
    if (_flags.reset_rate_to_accel_x) {
        // Reset Filter
        _vel_error.x = 0;
        _vel_x_error_filter.reset(0);
        _flags.reset_rate_to_accel_x = false;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        _vel_error.x = _vel_x_error_filter.apply(_vel_target.x - curr_vel.x, _dt);
    }

	if(print_flag) {
		hal.shell->printf("vt[%.4f] vc[%.4f] ve[%.4f]\r\n", 
					_vel_target.x,
					curr_vel.x,
					_vel_error.x);
	}
	
    _accel_target.x = _p_vel_x.get_p(_vel_error.x);

    _accel_target.x += _accel_desired.x;


    // the following section calculates a desired throttle needed to achieve the acceleration target
    float x_accel_meas;         // actual acceleration

    // Calculate Earth Frame X acceleration
    x_accel_meas = _ahrs.get_accel_ef_blended().x * 100.0f;

    float pid_out, out_filtered;
    pid_out = -_pid_accel_x.update_all(_accel_target.x, x_accel_meas, false) * 0.001f;
    if(print_flag) {
		hal.shell->printf("at[%.4f] ac[%.4f] ae[%.4f] pout[%.4f]\r\n", 
					_accel_target.x,
					x_accel_meas,
					_accel_target.x - x_accel_meas,
					pid_out);
	}

	Vector3f accel_meas = _ahrs.get_accel_ef_blended();
	accl_cm.x = accel_meas.x * 100.0f;
	accl_cm.y = accel_meas.y * 100.0f;
	accl_cm.z = -(accel_meas.z + GRAVITY_MSS) * 100.0f;
	if(print_flag) {
		hal.shell->printf("ax[%.4f] ay[%.4f] az[%.4f]\r\n", 
					accel_meas.x,
					accel_meas.y,
					accel_meas.z);
	}

	out_filtered = _out_x_filter.apply(pid_out, _dt);
    _motors.set_forward(out_filtered);
    
    print_flag = 0;
}

void DistanceControl::relax_y_controllers(float distance)
{
    _pos_target.y = distance;
    _vel_desired.y = 0.0f;
    _flags.use_desvel_ff_y = false;
    _vel_target.y = _inav.get_velocity().y;
    _vel_last.y = _vel_target.y;
    _accel_desired.y = 0.0f;
    _flags.reset_rate_to_accel_y = true;
    _accel_target.y = _ahrs.get_accel_ef_blended().y * 100.0f;
    _pid_accel_y.reset_filter();
}

void DistanceControl::update_y_controller(float distance)
{
    float curr_dis = distance;
    static uint8_t print_flag = 0;

    if(1) {
		static uint32_t _startup_ms = 0;

		if(_startup_ms == 0) {
			_startup_ms = AP_HAL::millis();
		}

		if(AP_HAL::millis() - _startup_ms > 1000) {
			_startup_ms = AP_HAL::millis();
			
			print_flag = 1;
		}
	}

    // calculate altitude error
    _pos_error.y = _pos_target.y - curr_dis;

    if(print_flag) {
		hal.shell->printf("pt[%.4f] pc[%.4f] pe[%.4f]\r\n", 
					_pos_target.y,
					curr_dis,
					_pos_error.y);
	}

    // calculate _vel_target.y using from _pos_error.y using sqrt controller
    _vel_target.y = AC_AttitudeControl::sqrt_controller(_pos_error.y, _p_pos_y.kP(), _accel_cms.y, _dt);

    // add feed forward component
    if (_flags.use_desvel_ff_y) {
        _vel_target.y += _vel_desired.y;
    }

    // the following section calculates acceleration required to achieve the velocity target

    const Vector3f& curr_vel = _inav.get_velocity();

    // TODO: remove velocity derivative calculation
    // reset last velocity target to current target
    if (_flags.reset_rate_to_accel_y) {
        _vel_last.y = _vel_target.y;
    }

    // feed forward desired acceleration calculation
    if (_dt > 0.0f) {
        if (!_flags.freeze_ff_y) {
            _accel_desired.y = (_vel_target.y - _vel_last.y) / _dt;
        } else {
            // stop the feed forward being calculated during a known discontinuity
            _flags.freeze_ff_y = false;
        }
    } else {
        _accel_desired.y = 0.0f;
    }

    // store this iteration's velocities for the next iteration
    _vel_last.y = _vel_target.y;

    // reset velocity error and filter if this controller has just been engaged
    if (_flags.reset_rate_to_accel_y) {
        // Reset Filter
        _vel_error.y = 0;
        _vel_y_error_filter.reset(0);
        _flags.reset_rate_to_accel_y = false;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        _vel_error.y = _vel_y_error_filter.apply(_vel_target.y - curr_vel.y, _dt);
    }

	if(print_flag) {
		hal.shell->printf("vt[%.4f] vc[%.4f] ve[%.4f]\r\n", 
					_vel_target.y,
					curr_vel.y,
					_vel_error.y);
	}
	
    _accel_target.y = _p_vel_y.get_p(_vel_error.y);

    _accel_target.y += _accel_desired.y;


    // the following section calculates a desired throttle needed to achieve the acceleration target
    float y_accel_meas;         // actual acceleration

    // Calculate Earth Frame Y acceleration
    y_accel_meas = _ahrs.get_accel_ef_blended().y * 100.0f;

    float pid_out, out_filtered;
    pid_out = -_pid_accel_y.update_all(_accel_target.y, y_accel_meas, false) * 0.001f;
    if(print_flag) {
		hal.shell->printf("at[%.4f] ac[%.4f] ae[%.4f] pout[%.4f]\r\n", 
					_accel_target.y,
					y_accel_meas,
					_accel_target.y - y_accel_meas,
					pid_out);
	}

	Vector3f accel_meas = _ahrs.get_accel_ef_blended();
	accl_cm.x = accel_meas.x * 100.0f;
	accl_cm.y = accel_meas.y * 100.0f;
	accl_cm.z = -(accel_meas.z + GRAVITY_MSS) * 100.0f;
	if(print_flag) {
		hal.shell->printf("ax[%.4f] ay[%.4f] az[%.4f]\r\n", 
					accel_meas.x,
					accel_meas.y,
					accel_meas.z);
	}

	out_filtered = _out_y_filter.apply(pid_out, _dt);
    _motors.set_lateral(out_filtered);
    
    print_flag = 0;
}


