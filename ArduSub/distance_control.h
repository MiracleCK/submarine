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


// position controller default definitions
#define DISCONTROL_ACCELERATION_MIN             50.0f   // minimum horizontal acceleration in cm/s/s - used for sanity checking acceleration in leash length calculation
#define DISCONTROL_ACCEL_XY                     100.0f  // default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
#define DISCONTROL_ACCEL_XY_MAX                 980.0f  // max horizontal acceleration in cm/s/s that the position velocity controller will ask from the lower accel controller
#define DISCONTROL_STOPPING_DIST_UP_MAX         300.0f  // max stopping distance (in cm) vertically while climbing
#define DISCONTROL_STOPPING_DIST_DOWN_MAX       200.0f  // max stopping distance (in cm) vertically while descending

#define DISCONTROL_ACCEL_X                      250.0f  // default x acceleration in cm/s/s.
#define DISCONTROL_ACCEL_Y                      250.0f  // default y acceleration in cm/s/s.
#define DISCONTROL_ACCEL_Z                      250.0f  // default z acceleration in cm/s/s.

#define DISCONTROL_DT_50HZ                      0.02f   // time difference in seconds for 50hz update rate
#define DISCONTROL_DT_400HZ                     0.0025f // time difference in seconds for 400hz update rate

#define DISCONTROL_ACTIVE_TIMEOUT_US            200000  // position controller is considered active if it has been called within the past 0.2 seconds

#define DISCONTROL_VEL_ERROR_CUTOFF_FREQ        4.0f    // low-pass filter on velocity error (unit: hz)
#define DISCONTROL_THROTTLE_CUTOFF_FREQ         2.0f    // low-pass filter on accel error (unit: hz)
#define DISCONTROL_JERK_RATIO                   1.0f    // Defines the time it takes to reach the requested acceleration

#define DISCONTROL_OVERSPEED_GAIN_Z             2.0f    // gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range

class DistanceControl
{
public:

    /// Constructor
    DistanceControl(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                  AP_Motors& motors, AC_AttitudeControl& attitude_control, const RangeFinder& rangefinder);

    ///
    /// initialisation functions
    ///

    /// set_dt - sets time delta in seconds for all controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    ///     updates z axis accel controller's D term filter
    void set_dt(float delta_sec);
    float get_dt() const { return _dt; }

    ///
    /// z position controller
    ///

    /// relax_alt_hold_controllers - set all desired and targets to measured
    void relax_z_controllers(float distance);
    void relax_x_controllers(float distance);
    void relax_y_controllers(float distance);

    /// get_z_target - get desired altitude (in cm above home) from loiter or wp controller which should be fed into throttle controller
    float get_z_target() const { return _pos_target.z; }

    /// update_z_controller - fly to altitude in cm above home
    void update_z_controller(float distance);
    void update_x_controller(float distance);
    void update_y_controller(float distance);

    /// get_desired_velocity - returns xy desired velocity (i.e. feed forward) in cm/s in lat and lon direction
    const Vector3f& get_desired_velocity() { return _vel_desired; }

    /// set_desired_velocity_z - sets desired velocity in cm/s in z axis
    void set_desired_velocity_z(float vel_z_cms) {_vel_desired.z = vel_z_cms;}

    // clear desired velocity feed-forward in z axis
    void clear_desired_velocity_ff_z() { _flags.use_desvel_ff_z = false; }

	void update_distance();

    /// get pid controllers
    AC_P& get_pos_z_p() { return _p_pos_z; }
    AC_P& get_vel_z_p() { return _p_vel_z; }
    AC_PID& get_accel_z_pid() { return _pid_accel_z; }

    static const struct AP_Param::GroupInfo var_info[];

protected:

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


    // references to inertial nav and ahrs libraries
    const AP_AHRS_View &        _ahrs;
    const AP_InertialNav&       _inav;
    AP_Motors&            _motors;
    const RangeFinder& 			_rangefinder;
    AC_AttitudeControl&         _attitude_control;

    // parameters
    AP_Float    _accel_xy_filt_hz;      // XY acceleration filter cutoff frequency
    AP_Float    _lean_angle_max;        // Maximum autopilot commanded angle (in degrees). Set to zero for Angle Max

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
    Vector3f    _accel_cms;           // max acceleration in cm/s/s
    
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
    Vector2f    _vehicle_horiz_vel;     // velocity to use if _flags.vehicle_horiz_vel_override is set
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

    AP_Int8     _hold_enable;
	AP_Int8     _fix_enable;
    AP_Int8 	_offset_front;
    AP_Int8 	_offset_back;
    AP_Int8 	_offset_left;
    AP_Int8 	_offset_right;
    
    AP_Int8 	_front_cm;
    AP_Int8 	_back_cm;
    AP_Int8 	_left_cm;
    AP_Int8 	_right_cm;
    AP_Int8 	_bottom_cm;
};
