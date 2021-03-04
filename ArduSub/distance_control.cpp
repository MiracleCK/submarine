#include <AP_HAL/AP_HAL.h>
#include "distance_control.h"
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // default gains for Plane
 # define DISCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define DISCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define DISCONTROL_ACC_Z_P                    0.3f    // vertical acceleration controller P gain default
 # define DISCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define DISCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define DISCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define DISCONTROL_ACC_Z_FILT_HZ              10.0f   // vertical acceleration controller input filter default
 # define DISCONTROL_ACC_Z_DT                   0.02f   // vertical acceleration controller dt default
 # define DISCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define DISCONTROL_VEL_XY_P                   1.4f    // horizontal velocity controller P gain default
 # define DISCONTROL_VEL_XY_I                   0.7f    // horizontal velocity controller I gain default
 # define DISCONTROL_VEL_XY_D                   0.35f   // horizontal velocity controller D gain default
 # define DISCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define DISCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define DISCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#elif APM_BUILD_TYPE(APM_BUILD_ArduSub)
 // default gains for Sub
 # define DISCONTROL_POS_Z_P                    3.0f    // vertical position controller P gain default
 # define DISCONTROL_VEL_Z_P                    8.0f    // vertical velocity controller P gain default
 # define DISCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define DISCONTROL_ACC_Z_I                    0.1f    // vertical acceleration controller I gain default
 # define DISCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define DISCONTROL_ACC_Z_IMAX                 100     // vertical acceleration controller IMAX gain default
 # define DISCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define DISCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define DISCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define DISCONTROL_VEL_XY_P                   1.0f    // horizontal velocity controller P gain default
 # define DISCONTROL_VEL_XY_I                   0.5f    // horizontal velocity controller I gain default
 # define DISCONTROL_VEL_XY_D                   0.0f    // horizontal velocity controller D gain default
 # define DISCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define DISCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define DISCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#else
 // default gains for Copter / TradHeli
 # define DISCONTROL_POS_Z_P                    1.0f    // vertical position controller P gain default
 # define DISCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
 # define DISCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
 # define DISCONTROL_ACC_Z_I                    1.0f    // vertical acceleration controller I gain default
 # define DISCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
 # define DISCONTROL_ACC_Z_IMAX                 800     // vertical acceleration controller IMAX gain default
 # define DISCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
 # define DISCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default
 # define DISCONTROL_POS_XY_P                   1.0f    // horizontal position controller P gain default
 # define DISCONTROL_VEL_XY_P                   2.0f    // horizontal velocity controller P gain default
 # define DISCONTROL_VEL_XY_I                   1.0f    // horizontal velocity controller I gain default
 # define DISCONTROL_VEL_XY_D                   0.5f    // horizontal velocity controller D gain default
 # define DISCONTROL_VEL_XY_IMAX                1000.0f // horizontal velocity controller IMAX gain default
 # define DISCONTROL_VEL_XY_FILT_HZ             5.0f    // horizontal velocity controller input filter
 # define DISCONTROL_VEL_XY_FILT_D_HZ           5.0f    // horizontal velocity controller input filter for D
#endif

// vibration compensation gains
#define DISCONTROL_VIBE_COMP_P_GAIN 0.250f
#define DISCONTROL_VIBE_COMP_I_GAIN 0.125f

const AP_Param::GroupInfo DistanceControl::var_info[] = {
    // 0 was used for HOVER

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

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
DistanceControl::DistanceControl(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                             AP_Motors& motors, AC_AttitudeControl& attitude_control) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control),
    _p_pos_z(DISCONTROL_POS_Z_P),
    _p_vel_z(DISCONTROL_VEL_Z_P),
    _pid_accel_z(DISCONTROL_ACC_Z_P, DISCONTROL_ACC_Z_I, DISCONTROL_ACC_Z_D, 0.0f, DISCONTROL_ACC_Z_IMAX, 0.0f, DISCONTROL_ACC_Z_FILT_HZ, 0.0f, DISCONTROL_ACC_Z_DT),
    _p_pos_xy(DISCONTROL_POS_XY_P),
    _pid_vel_xy(DISCONTROL_VEL_XY_P, DISCONTROL_VEL_XY_I, DISCONTROL_VEL_XY_D, DISCONTROL_VEL_XY_IMAX, DISCONTROL_VEL_XY_FILT_HZ, DISCONTROL_VEL_XY_FILT_D_HZ, DISCONTROL_DT_50HZ),
    _dt(DISCONTROL_DT_400HZ),
    _speed_down_cms(DISCONTROL_SPEED_DOWN),
    _speed_up_cms(DISCONTROL_SPEED_UP),
    _speed_cms(DISCONTROL_SPEED),
    _accel_z_cms(DISCONTROL_ACCEL_Z),
    _accel_cms(DISCONTROL_ACCEL_XY),
    _leash(DISCONTROL_LEASH_LENGTH_MIN),
    _leash_down_z(DISCONTROL_LEASH_LENGTH_MIN),
    _leash_up_z(DISCONTROL_LEASH_LENGTH_MIN),
    _accel_target_filter(DISCONTROL_ACCEL_FILTER_HZ)
{
    AP_Param::setup_object_defaults(this, var_info);

    // initialise flags
    _flags.recalc_leash_z = true;
    _flags.recalc_leash_xy = true;
    _flags.reset_desired_vel_to_pos = true;
    _flags.reset_accel_to_lean_xy = true;
    _flags.reset_rate_to_accel_z = true;
    _flags.freeze_ff_z = true;
    _flags.use_desvel_ff_z = true;
    _limit.pos_up = true;
    _limit.pos_down = true;
    _limit.vel_up = true;
    _limit.vel_down = true;
    _limit.accel_xy = true;
}

void DistanceControl::relax_alt_hold_controllers(float distance)
{
	//hal.shell->printf(">>set target %f\r\n", distance);
    _pos_target.z = distance;
    _vel_desired.z = 0.0f;
    _flags.use_desvel_ff_z = false;
    _vel_target.z = _inav.get_velocity_z();
    _vel_last.z = _inav.get_velocity_z();
    _accel_desired.z = 0.0f;
    _accel_last_z_cms = 0.0f;
    _flags.reset_rate_to_accel_z = true;
    //_pid_accel_z.set_integrator((throttle_setting - _motors.get_throttle_hover()) * 1000.0f);
    _accel_target.z = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;
    _pid_accel_z.reset_filter();
}

#if 0
void DistanceControl::update_z_controller(float distance)
{
    float curr_alt = -distance;

    // calculate altitude error
    _pos_error.z = _pos_target.z - curr_alt;

    // calculate _vel_target.z using from _pos_error.z using sqrt controller
    _vel_target.z = AC_AttitudeControl::sqrt_controller(_pos_error.z, _p_pos_z.kP(), _accel_z_cms, _dt);

    // check speed limits
    // To-Do: check these speed limits here or in the pos->rate controller
    _limit.vel_up = false;
    _limit.vel_down = false;
    if (_vel_target.z < _speed_down_cms) {
        _vel_target.z = _speed_down_cms;
        _limit.vel_down = true;
    }
    if (_vel_target.z > _speed_up_cms) {
        _vel_target.z = _speed_up_cms;
        _limit.vel_up = true;
    }

    // add feed forward component
    if (_flags.use_desvel_ff_z) {
        _vel_target.z += _vel_desired.z;
    }

    // the following section calculates acceleration required to achieve the velocity target

    const Vector3f& curr_vel = _inav.get_velocity();

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
        _vel_error_filter.reset(0);
        _flags.reset_rate_to_accel_z = false;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        _vel_error.z = _vel_error_filter.apply(_vel_target.z - curr_vel.z, _dt);
    }

    _accel_target.z = _p_vel_z.get_p(_vel_error.z);

    _accel_target.z += _accel_desired.z;


    // the following section calculates a desired throttle needed to achieve the acceleration target
    float z_accel_meas;         // actual acceleration

    // Calculate Earth Frame Z acceleration
    z_accel_meas = -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f;

    // ensure imax is always large enough to overpower hover throttle
    if (_motors.get_throttle_hover() * 1000.0f > _pid_accel_z.imax()) {
        _pid_accel_z.imax(_motors.get_throttle_hover() * 1000.0f);
    }

    float thr_out;
    if (_vibe_comp_enabled) {
        _flags.freeze_ff_z = true;
        _accel_desired.z = 0.0f;
        const float thr_per_accelz_cmss = _motors.get_throttle_hover() / (GRAVITY_MSS * 100.0f);
        // during vibration compensation use feed forward with manually calculated gain
        // ToDo: clear pid_info P, I and D terms for logging
        if (!(_motors.limit.throttle_lower || _motors.limit.throttle_upper) || ((is_positive(_pid_accel_z.get_i()) && is_negative(_vel_error.z)) || (is_negative(_pid_accel_z.get_i()) && is_positive(_vel_error.z)))) {
            _pid_accel_z.set_integrator(_pid_accel_z.get_i() + _dt * thr_per_accelz_cmss * 1000.0f * _vel_error.z * _p_vel_z.kP() * DISCONTROL_VIBE_COMP_I_GAIN);
        }
        thr_out = DISCONTROL_VIBE_COMP_P_GAIN * thr_per_accelz_cmss * _accel_target.z + _pid_accel_z.get_i() * 0.001f;
    } else {
        thr_out = _pid_accel_z.update_all(_accel_target.z, z_accel_meas, (_motors.limit.throttle_lower || _motors.limit.throttle_upper)) * 0.001f;
    }

    if(1) {
		static uint32_t _startup_ms = 0;

		if(_startup_ms == 0) {
			_startup_ms = AP_HAL::millis();
		}

		if(AP_HAL::millis() - _startup_ms > 1000) {
			_startup_ms = AP_HAL::millis();
			
			hal.shell->printf("%f %f\r\n", 
						thr_out,
						_motors.get_throttle_hover());
		}
	}
	
    thr_out += _motors.get_throttle_hover();

    // send throttle to attitude controller with angle boost
    _attitude_control.set_throttle_out(thr_out, true, DISCONTROL_THROTTLE_CUTOFF_FREQ);

    // _speed_down_cms is checked to be non-zero when set
    float error_ratio = _vel_error.z/_speed_down_cms;

    _vel_z_control_ratio += _dt*0.1f*(0.5-error_ratio);
    _vel_z_control_ratio = constrain_float(_vel_z_control_ratio, 0.0f, 2.0f);
}

#endif

float vel_cm, accl_cm;
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
    _vel_target.z = AC_AttitudeControl::sqrt_controller(_pos_error.z, _p_pos_z.kP(), _accel_z_cms, _dt);

    // check speed limits
    // To-Do: check these speed limits here or in the pos->rate controller
    _limit.vel_up = false;
    _limit.vel_down = false;
    if (_vel_target.z < _speed_down_cms) {
        _vel_target.z = _speed_down_cms;
        _limit.vel_down = true;
    }
    if (_vel_target.z > _speed_up_cms) {
        _vel_target.z = _speed_up_cms;
        _limit.vel_up = true;
    }

    // add feed forward component
    if (_flags.use_desvel_ff_z) {
        _vel_target.z += _vel_desired.z;
    }

    // the following section calculates acceleration required to achieve the velocity target

    const Vector3f& curr_vel = _inav.get_velocity();
    vel_cm = curr_vel.z;

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
        _vel_error_filter.reset(0);
        _flags.reset_rate_to_accel_z = false;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        _vel_error.z = _vel_error_filter.apply(_vel_target.z - curr_vel.z, _dt);
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
	accl_cm = z_accel_meas;
	
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
	//_motors.set_throttle(thr_out);
	
    // _speed_down_cms is checked to be non-zero when set
    float error_ratio = _vel_error.z/_speed_down_cms;

    _vel_z_control_ratio += _dt*0.1f*(0.5-error_ratio);
    _vel_z_control_ratio = constrain_float(_vel_z_control_ratio, 0.0f, 2.0f);

    print_flag = 0;
}


