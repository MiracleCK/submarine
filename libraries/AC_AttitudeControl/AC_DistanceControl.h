#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_P.h>               // P library
#include <AC_PID/AC_PID.h>             // PID library
#include <AC_PID/AC_PI_2D.h>           // PI library (2-axis)
#include <AC_PID/AC_PID_2D.h>          // PID library (2-axis)
#include <AP_InertialNav/AP_InertialNav.h>     // Inertial Navigation library
#include <AC_AttitudeControl/AC_AttitudeControl.h> // Attitude control library
#include <AP_Motors/AP_Motors.h>          // motors library
#include <AP_Vehicle/AP_Vehicle.h>         // common vehicle parameters

class AC_DistanceControl
{
public:

    /// Constructor
    AC_DistanceControl(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                  AP_Motors& motors, AC_AttitudeControl& attitude_control, const RangeFinder& rangefinder);


    /// relax_z_controller - set all desired and targets to measured
    void relax_z_controller(float distance);
    void relax_x_controller(float distance);
    void relax_y_controller(float distance);

    /// update_z_controller - fly to altitude in cm above home
    void update_z_controller(float distance);
    void update_x_controller(float distance);
    void update_y_controller(float distance);

	float get_target_z() const { return _pos_target.z; }
    float get_target_x() const { return _pos_target.x; }
	float get_target_y() const { return _pos_target.y; }
	
	bool limit_enable() const { return _limit_enable.get(); }
	
    bool front_face_is_active() const { return (_distance_face.get() & 0x01); }
    bool back_face_is_active() const { return (_distance_face.get() & 0x02); }
    bool left_face_is_active() const { return (_distance_face.get() & 0x04); }
    bool right_face_is_active() const { return (_distance_face.get() & 0x08); }
    bool top_face_is_active() const { return (_distance_face.get() & 0x10); }
    bool bottom_face_is_active() const { return (_distance_face.get() & 0x20); }

    int16_t get_front_cm() const { return distance_ned[DISTANCE_FRONT]; }
    int16_t get_back_cm() const { return distance_ned[DISTANCE_BACK]; }
    int16_t get_left_cm() const { return distance_ned[DISTANCE_LEFT]; }
    int16_t get_right_cm() const { return distance_ned[DISTANCE_RIGHT]; }
    int16_t get_top_cm() const { return distance_ned[DISTANCE_TOP]; }
    int16_t get_bottom_cm() const { return distance_ned[DISTANCE_BOTTOM]; }

    int16_t get_front_cm_bf() const { return distance_bf[DISTANCE_FRONT]; }
    int16_t get_back_cm_bf() const { return distance_bf[DISTANCE_BACK]; }
    int16_t get_left_cm_bf() const { return distance_bf[DISTANCE_LEFT]; }
    int16_t get_right_cm_bf() const { return distance_bf[DISTANCE_RIGHT]; }
    int16_t get_top_cm_bf() const { return distance_bf[DISTANCE_TOP]; }
    int16_t get_bottom_cm_bf() const { return distance_bf[DISTANCE_BOTTOM]; }

    int16_t get_front_limit_cm() const { return _front_limit_cm.get(); }
    int16_t get_back_limit_cm() const { return _back_limit_cm.get(); }
    int16_t get_left_limit_cm() const { return _left_limit_cm.get(); }
    int16_t get_right_limit_cm() const { return _right_limit_cm.get(); }
    int16_t get_top_limit_cm() const { return _top_limit_cm.get(); }
    int16_t get_bottom_limit_cm() const { return _bottom_limit_cm.get(); }

    int16_t get_delayms_x() const { return _delay_ms_x.get(); }
    int16_t get_delayms_y() const { return _delay_ms_y.get(); }
    int16_t get_delayms_z() const { return _delay_ms_z.get(); }
    
	void update_backend(Vector3f &thrusts);
	static AC_DistanceControl *get_singleton(void) { return _singleton; }
	
    static const struct AP_Param::GroupInfo var_info[];
	
private:
	static AC_DistanceControl *_singleton;

    // general purpose flags
    struct DISCONTROL_flags {
    		uint16_t reset_rate_to_accel_x      : 1;    // 1 if we should reset the rate_to_accel_z step
            uint16_t freeze_ff_x        : 1;    // 1 used to freeze velocity to accel feed forward for one iteration
            uint16_t use_desvel_ff_x    : 1;    // 1 to use z-axis desired velocity as feed forward into velocity step

            uint16_t reset_rate_to_accel_y      : 1;    // 1 if we should reset the rate_to_accel_z step
            uint16_t freeze_ff_y        : 1;    // 1 used to freeze velocity to accel feed forward for one iteration
            uint16_t use_desvel_ff_y    : 1;    // 1 to use z-axis desired velocity as feed forward into velocity step
            
            uint16_t reset_rate_to_accel_z      : 1;    // 1 if we should reset the rate_to_accel_z step
            uint16_t freeze_ff_z        : 1;    // 1 used to freeze velocity to accel feed forward for one iteration
            uint16_t use_desvel_ff_z    : 1;    // 1 to use z-axis desired velocity as feed forward into velocity step
    } _flags;

    /// Proportional controller with piecewise sqrt sections to constrain second derivative
    static Vector3f sqrt_controller(const Vector3f& error, float p, float second_ord_lim);
	void update_distance();
	void pilot_thrusts_scale(Vector3f &thrusts);
	void pilot_thrusts_limit(Vector3f &thrusts);
	void attitude_filter(Vector3f &thrusts);

    // references to inertial nav and ahrs libraries
    const AP_AHRS_View &        _ahrs;
    const AP_InertialNav&       _inav;
    AP_Motors&            _motors;
    const RangeFinder& 			_rangefinder;
    AC_AttitudeControl&         _attitude_control;

	AC_P        _p_pos_x;
    AC_P        _p_vel_x;
    AC_PID      _pid_accel_x;
    
    AC_P        _p_pos_y;
    AC_P        _p_vel_y;
    AC_PID      _pid_accel_y;
    
    AC_P        _p_pos_z;
    AC_P        _p_vel_z;
    AC_PID      _pid_accel_z;
	
    // internal variables
    float       _dt;                    // time difference (in seconds) between calls from the main program
    
    // position controller internal variables
    Vector3f    _pos_target;            // target location in cm from home
    Vector3f    _pos_error;             // error between desired and actual position in cm
    Vector3f    _vel_desired;           // desired velocity in cm/s
    Vector3f    _vel_target;            // velocity target in cm/s calculated by pos_to_rate step
    Vector3f    _vel_error;             // error between desired and actual acceleration in cm/s
    Vector3f    _vel_last;              // previous iterations velocity in cm/s
    Vector3f    _accel_desired;         // desired acceleration in cm/s/s (feed forward)
    Vector3f    _accel_target;          // acceleration target in cm/s/s
    Vector3f    _accel_error;           // acceleration error in cm/s/s
    LowPassFilterFloat _vel_x_error_filter;   // low-pass-filter on z-axis velocity error
    LowPassFilterFloat _vel_y_error_filter;   // low-pass-filter on z-axis velocity error
    LowPassFilterFloat _vel_z_error_filter;   // low-pass-filter on z-axis velocity error
	LowPassFilterFloat  _out_x_filter; 
	LowPassFilterFloat  _out_y_filter; 
	
    // high vibration handling
    bool        _vibe_comp_enabled;     // true when high vibration compensation is on

    enum distance_dir_t : uint8_t {
        DISTANCE_FRONT    = 0,
        DISTANCE_BACK     = 1,
        DISTANCE_LEFT     = 2,
        DISTANCE_RIGHT    = 3,
        DISTANCE_TOP   	  = 4,
        DISTANCE_BOTTOM   = 5,
        DISTANCE_NUM
    };
    int16_t distance_bf[DISTANCE_NUM];
    int16_t distance_ned[DISTANCE_NUM];

	AP_Float	_thr_face_p;
	AP_Float	_thr_limit_p;
	AP_Float	_limit_x_p;
	AP_Float	_limit_y_p;
	AP_Float	_limit_z_p;
    AP_Int8     _limit_enable;
	AP_Int8     _distance_face;
    AP_Int8 	_front_offset;
    AP_Int8 	_back_offset;
    AP_Int8 	_left_offset;
    AP_Int8 	_right_offset;
    
    AP_Int8 	_front_limit_cm;
    AP_Int8 	_back_limit_cm;
    AP_Int8 	_left_limit_cm;
    AP_Int8 	_right_limit_cm;
    AP_Int8 	_top_limit_cm;
    AP_Int8 	_bottom_limit_cm;

    AP_Int16 	_delay_ms_x;
    AP_Int16 	_delay_ms_y;
    AP_Int16 	_delay_ms_z;

    AP_Float	_max_accel_x;
	AP_Float	_max_accel_y;
	AP_Float	_max_accel_z;

	AP_Int8 	_curve_x;
    AP_Int8 	_curve_y;
    AP_Int8 	_curve_z;

    AP_Float	_max_speed_x;
    AP_Float	_max_speed_y;
    AP_Float	_max_speed_z;
};

namespace AP {
    AC_DistanceControl *distance_control();
};

