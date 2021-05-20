#include <AP_HAL/AP_HAL.h>
#include "AC_DistanceControl.h"
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

extern const AP_HAL::HAL& hal;

#define DISCONTROL_DT_50HZ                      0.02f   // time difference in seconds for 50hz update rate
#define DISCONTROL_DT_400HZ                     0.0025f // time difference in seconds for 400hz update rate

#define DISCONTROL_VEL_X_ERROR_CUTOFF_FREQ        4.0f    // low-pass filter on x error (unit: hz)
#define DISCONTROL_VEL_Y_ERROR_CUTOFF_FREQ        4.0f    // low-pass filter on y error (unit: hz)
#define DISCONTROL_VEL_Z_ERROR_CUTOFF_FREQ        4.0f    // low-pass filter on z error (unit: hz)

#define DISCONTROL_THROTTLE_CUTOFF_FREQ      2.0f    // low-pass filter on throttle error (unit: hz)
#define DISCONTROL_OUT_X_CUTOFF_FREQ         2.0f    // low-pass filter on output error (unit: hz)
#define DISCONTROL_OUT_Y_CUTOFF_FREQ         2.0f    // low-pass filter on output error (unit: hz)

#if 0 // default
// default gains for Sub
#define DISCONTROL_POS_Z_P                    3.0f    // vertical position controller P gain default
#define DISCONTROL_VEL_Z_P                    8.0f    // vertical velocity controller P gain default
#define DISCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
#define DISCONTROL_ACC_Z_I                    0.1f    // vertical acceleration controller I gain default
#define DISCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
#define DISCONTROL_ACC_Z_IMAX                 100     // vertical acceleration controller IMAX gain default
#define DISCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
#define DISCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default

#define DISCONTROL_POS_X_P                    3.0f    // vertical position controller P gain default
#define DISCONTROL_VEL_X_P                    8.0f    // vertical velocity controller P gain default
#define DISCONTROL_ACC_X_P                    0.5f    // vertical acceleration controller P gain default
#define DISCONTROL_ACC_X_I                    0.1f    // vertical acceleration controller I gain default
#define DISCONTROL_ACC_X_D                    0.0f    // vertical acceleration controller D gain default
#define DISCONTROL_ACC_X_IMAX                 100     // vertical acceleration controller IMAX gain default
#define DISCONTROL_ACC_X_FILT_HZ              20.0f   // vertical acceleration controller input filter default
#define DISCONTROL_ACC_X_DT                   0.0025f // vertical acceleration controller dt default

#define DISCONTROL_POS_Y_P                    3.0f    // vertical position controller P gain default
#define DISCONTROL_VEL_Y_P                    8.0f    // vertical velocity controller P gain default
#define DISCONTROL_ACC_Y_P                    0.5f    // vertical acceleration controller P gain default
#define DISCONTROL_ACC_Y_I                    0.1f    // vertical acceleration controller I gain default
#define DISCONTROL_ACC_Y_D                    0.0f    // vertical acceleration controller D gain default
#define DISCONTROL_ACC_Y_IMAX                 100     // vertical acceleration controller IMAX gain default
#define DISCONTROL_ACC_Y_FILT_HZ              20.0f   // vertical acceleration controller input filter default
#define DISCONTROL_ACC_Y_DT                   0.0025f // vertical acceleration controller dt default

#define DISCONTROL_LIMIT_X_P                  100.0f    // discontrol limit x P gain default
#define DISCONTROL_LIMIT_Y_P                  100.0f    // discontrol limit y P gain default
#define DISCONTROL_LIMIT_Z_P                  100.0f    // discontrol limit z P gain default

#define DISCONTROL_THRUSTS_P                  0.3f    // discontrol thrusts scale P gain default

#define DISCONTROL_FRONT_LIMIT_CM             30    // discontrol front limit default
#define DISCONTROL_BACK_LIMIT_CM             -30    // discontrol back limit default
#define DISCONTROL_LEFT_LIMIT_CM             -30    // discontrol left limit default
#define DISCONTROL_RIGHT_LIMIT_CM             30    // discontrol right limit default
#define DISCONTROL_TOP_LIMIT_CM              -30    // discontrol top limit default
#define DISCONTROL_BOTTOM_LIMIT_CM            30    // discontrol bottom limit default

#define DISCONTROL_FRONT_OFT_CM              16    // discontrol front offset default
#define DISCONTROL_BACK_OFT_CM               24    // discontrol back offset default
#define DISCONTROL_LEFT_OFT_CM                8    // discontrol left offset default
#define DISCONTROL_RIGHT_OFT_CM               8    // discontrol right offset default
#endif

#if 0  // ousailong
// default gains for Sub
#define DISCONTROL_POS_Z_P                    6.0f    // vertical position controller P gain default
#define DISCONTROL_VEL_Z_P                    8.0f    // vertical velocity controller P gain default
#define DISCONTROL_ACC_Z_P                    0.8f    // vertical acceleration controller P gain default
#define DISCONTROL_ACC_Z_I                    0.1f    // vertical acceleration controller I gain default
#define DISCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
#define DISCONTROL_ACC_Z_IMAX                 100     // vertical acceleration controller IMAX gain default
#define DISCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
#define DISCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default

#define DISCONTROL_POS_X_P                    5.0f    // vertical position controller P gain default
#define DISCONTROL_VEL_X_P                    5.0f    // vertical velocity controller P gain default
#define DISCONTROL_ACC_X_P                    0.8f    // vertical acceleration controller P gain default
#define DISCONTROL_ACC_X_I                    0.1f    // vertical acceleration controller I gain default
#define DISCONTROL_ACC_X_D                    0.0f    // vertical acceleration controller D gain default
#define DISCONTROL_ACC_X_IMAX                 100     // vertical acceleration controller IMAX gain default
#define DISCONTROL_ACC_X_FILT_HZ              20.0f   // vertical acceleration controller input filter default
#define DISCONTROL_ACC_X_DT                   0.0025f // vertical acceleration controller dt default

#define DISCONTROL_POS_Y_P                    6.0f    // vertical position controller P gain default
#define DISCONTROL_VEL_Y_P                    8.0f    // vertical velocity controller P gain default
#define DISCONTROL_ACC_Y_P                    0.8f    // vertical acceleration controller P gain default
#define DISCONTROL_ACC_Y_I                    0.1f    // vertical acceleration controller I gain default
#define DISCONTROL_ACC_Y_D                    0.0f    // vertical acceleration controller D gain default
#define DISCONTROL_ACC_Y_IMAX                 100     // vertical acceleration controller IMAX gain default
#define DISCONTROL_ACC_Y_FILT_HZ              20.0f   // vertical acceleration controller input filter default
#define DISCONTROL_ACC_Y_DT                   0.0025f // vertical acceleration controller dt default

#define DISCONTROL_LIMIT_X_P                  100.0f    // discontrol limit x P gain default
#define DISCONTROL_LIMIT_Y_P                  100.0f    // discontrol limit y P gain default
#define DISCONTROL_LIMIT_Z_P                  100.0f    // discontrol limit z P gain default

#define DISCONTROL_THRUSTS_P                  0.3f    // discontrol thrusts scale P gain default

#define DISCONTROL_FRONT_LIMIT_CM             30    // discontrol front limit default
#define DISCONTROL_BACK_LIMIT_CM             -30    // discontrol back limit default
#define DISCONTROL_LEFT_LIMIT_CM             -30    // discontrol left limit default
#define DISCONTROL_RIGHT_LIMIT_CM             30    // discontrol right limit default
#define DISCONTROL_TOP_LIMIT_CM              -30    // discontrol top limit default
#define DISCONTROL_BOTTOM_LIMIT_CM            30    // discontrol bottom limit default

#define DISCONTROL_FRONT_OFT_CM              16    // discontrol front offset default
#define DISCONTROL_BACK_OFT_CM               24    // discontrol back offset default
#define DISCONTROL_LEFT_OFT_CM                8    // discontrol left offset default
#define DISCONTROL_RIGHT_OFT_CM               8    // discontrol right offset default
#endif

#if 1  // dayu - 500K_100倍_5合1单前轴_35ms
#define DISCONTROL_POS_Z_P                    8.0f    // vertical position controller P gain default
#define DISCONTROL_VEL_Z_P                    5.0f    // vertical velocity controller P gain default
#define DISCONTROL_ACC_Z_P                    0.5f    // vertical acceleration controller P gain default
#define DISCONTROL_ACC_Z_I                    0.1f    // vertical acceleration controller I gain default
#define DISCONTROL_ACC_Z_D                    0.0f    // vertical acceleration controller D gain default
#define DISCONTROL_ACC_Z_IMAX                 200     // vertical acceleration controller IMAX gain default
#define DISCONTROL_ACC_Z_FILT_HZ              20.0f   // vertical acceleration controller input filter default
#define DISCONTROL_ACC_Z_DT                   0.0025f // vertical acceleration controller dt default

#define DISCONTROL_POS_X_P                    8.0f    // vertical position controller P gain default
#define DISCONTROL_VEL_X_P                    5.0f    // vertical velocity controller P gain default
#define DISCONTROL_ACC_X_P                    0.5f    // vertical acceleration controller P gain default
#define DISCONTROL_ACC_X_I                    0.1f    // vertical acceleration controller I gain default
#define DISCONTROL_ACC_X_D                    0.0f    // vertical acceleration controller D gain default
#define DISCONTROL_ACC_X_IMAX                 200     // vertical acceleration controller IMAX gain default
#define DISCONTROL_ACC_X_FILT_HZ              20.0f   // vertical acceleration controller input filter default
#define DISCONTROL_ACC_X_DT                   0.0025f // vertical acceleration controller dt default

#define DISCONTROL_POS_Y_P                    10.0f    // vertical position controller P gain default
#define DISCONTROL_VEL_Y_P                    3.0f    // vertical velocity controller P gain default
#define DISCONTROL_ACC_Y_P                    0.5f    // vertical acceleration controller P gain default
#define DISCONTROL_ACC_Y_I                    0.1f    // vertical acceleration controller I gain default
#define DISCONTROL_ACC_Y_D                    0.0f    // vertical acceleration controller D gain default
#define DISCONTROL_ACC_Y_IMAX                 200     // vertical acceleration controller IMAX gain default
#define DISCONTROL_ACC_Y_FILT_HZ              20.0f   // vertical acceleration controller input filter default
#define DISCONTROL_ACC_Y_DT                   0.0025f // vertical acceleration controller dt default

#define DISCONTROL_LIMIT_X_P                  80.0f    // discontrol limit x P gain default
#define DISCONTROL_LIMIT_Y_P                  80.0f    // discontrol limit y P gain default
#define DISCONTROL_LIMIT_Z_P                  80.0f    // discontrol limit z P gain default

#define DISCONTROL_THRUSTS_FACE_P             1.0f //0.3f    // discontrol face thrusts scale P gain default
#define DISCONTROL_THRUSTS_LIMIT_P            1.0f //0.8f    // discontrol limit thrusts scale P gain default

#define DISCONTROL_FRONT_LIMIT_CM             30    // discontrol front limit default
#define DISCONTROL_BACK_LIMIT_CM             -30    // discontrol back limit default
#define DISCONTROL_LEFT_LIMIT_CM             -30    // discontrol left limit default
#define DISCONTROL_RIGHT_LIMIT_CM             30    // discontrol right limit default
#define DISCONTROL_TOP_LIMIT_CM              -0    // discontrol top limit default
#define DISCONTROL_BOTTOM_LIMIT_CM            30    // discontrol bottom limit default

#define DISCONTROL_FRONT_OFT_CM               0  //16    // discontrol front offset default
#define DISCONTROL_BACK_OFT_CM                0  //24    // discontrol back offset default
#define DISCONTROL_LEFT_OFT_CM                8    // discontrol left offset default
#define DISCONTROL_RIGHT_OFT_CM               8    // discontrol right offset default
#define DISCONTROL_FRONT347_OFT_CM            18    // discontrol front347 offset default
#define DISCONTROL_FRONT13_OFT_CM             18    // discontrol front13 offset default

#define DISCONTROL_DELAY_MS_X                 1000    // discontrol x delay time
#define DISCONTROL_DELAY_MS_Y                 1500    // discontrol y delay time
#define DISCONTROL_DELAY_MS_Z                 1000    // discontrol z delay time

#define DISCONTROL_MAX_ACCEL_X                500.0f  // default x acceleration in cm/s/s.
#define DISCONTROL_MAX_ACCEL_Y                500.0f  // default y acceleration in cm/s/s.
#define DISCONTROL_MAX_ACCEL_Z                500.0f  // default z acceleration in cm/s/s.

#define DISCONTROL_CURVE_X                    2  // default x curve
#define DISCONTROL_CURVE_Y                    2  // default y curve
#define DISCONTROL_CURVE_Z                    2  // default z curve

#define DISCONTROL_MAX_SPEED_X                0 //150.0f  // default x speed in cm/s.
#define DISCONTROL_MAX_SPEED_Y                0 //150.0f  // default y speed in cm/s.
#define DISCONTROL_MAX_SPEED_Z                0 //150.0f  // default z speed in cm/s.

#endif

// vibration compensation gains
#define DISCONTROL_VIBE_COMP_P_GAIN 0.250f
#define DISCONTROL_VIBE_COMP_I_GAIN 0.125f

#if !defined(MAX)
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif
#if !defined(MIN)
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

const AP_Param::GroupInfo AC_DistanceControl::var_info[] = {
   	// @Param: _POSZ_P
    // @DisplayName: Position (vertical) controller P gain
    // @Description: Position (vertical) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_z, "_POSZ_", 1, AC_DistanceControl, AC_P),

    // @Param: _VELZ_P
    // @DisplayName: Velocity (vertical) controller P gain
    // @Description: Velocity (vertical) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_vel_z, "_VELZ_", 2, AC_DistanceControl, AC_P),

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
    AP_SUBGROUPINFO(_pid_accel_z, "_ACCZ_", 3, AC_DistanceControl, AC_PID),

    // @Param: _POSX_P
    // @DisplayName: Position (X) controller P gain
    // @Description: Position (X) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_x, "_POSX_", 4, AC_DistanceControl, AC_P),

    // @Param: _VELX_P
    // @DisplayName: Velocity (X) controller P gain
    // @Description: Velocity (X) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_vel_x, "_VELX_", 5, AC_DistanceControl, AC_P),

    // @Param: _ACCX_P
    // @DisplayName: Acceleration (X) controller P gain
    // @Description: Acceleration (X) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @Increment: 0.05
    // @User: Standard

    // @Param: _ACCX_I
    // @DisplayName: Acceleration (X) controller I gain
    // @Description: Acceleration (X) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: _ACCX_IMAX
    // @DisplayName: Acceleration (X) controller I gain maximum
    // @Description: Acceleration (X) controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: d%
    // @User: Standard

    // @Param: _ACCX_D
    // @DisplayName: Acceleration (X) controller D gain
    // @Description: Acceleration (X) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACCX_FILT
    // @DisplayName: Acceleration (X) controller filter
    // @Description: Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_accel_x, "_ACCX_", 6, AC_DistanceControl, AC_PID),
    
    // @Param: _POSY_P
    // @DisplayName: Position (Y) controller P gain
    // @Description: Position (Y) controller P gain.  Converts the difference between the desired altitude and actual altitude into a climb or descent rate which is passed to the throttle rate controller
    // @Range: 1.000 3.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_pos_y, "_POSY_", 7, AC_DistanceControl, AC_P),

    // @Param: _VELY_P
    // @DisplayName: Velocity (Y) controller P gain
    // @Description: Velocity (Y) controller P gain.  Converts the difference between desired vertical speed and actual speed into a desired acceleration that is passed to the throttle acceleration controller
    // @Range: 1.000 8.000
    // @User: Standard
    AP_SUBGROUPINFO(_p_vel_y, "_VELY_", 8, AC_DistanceControl, AC_P),

    // @Param: _ACCY_P
    // @DisplayName: Acceleration (Y) controller P gain
    // @Description: Acceleration (Y) controller P gain.  Converts the difference between desired vertical acceleration and actual acceleration into a motor output
    // @Range: 0.500 1.500
    // @Increment: 0.05
    // @User: Standard

    // @Param: _ACCY_I
    // @DisplayName: Acceleration (Y) controller I gain
    // @Description: Acceleration (Y) controller I gain.  Corrects long-term difference in desired vertical acceleration and actual acceleration
    // @Range: 0.000 3.000
    // @User: Standard

    // @Param: _ACCY_IMAX
    // @DisplayName: Acceleration (Y) controller I gain maximum
    // @Description: Acceleration (Y) controller I gain maximum.  Constrains the maximum pwm that the I term will generate
    // @Range: 0 1000
    // @Units: d%
    // @User: Standard

    // @Param: _ACCY_D
    // @DisplayName: Acceleration (Y) controller D gain
    // @Description: Acceleration (Y) controller D gain.  Compensates for short-term change in desired vertical acceleration vs actual acceleration
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _ACCY_FILT
    // @DisplayName: Acceleration (Y) controller filter
    // @Description: Filter applied to acceleration to reduce noise.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_pid_accel_y, "_ACCY_", 9, AC_DistanceControl, AC_PID),

    // @Param: _LIMIT_X_P
    // @DisplayName: distance x limit P gain
    // @Description: Less than how many centimeters for speed limit
    // @User: Standard
    // @units: cm
    // @Values: > 0
    AP_GROUPINFO("_LIMIT_X_P",  10, AC_DistanceControl, _limit_x_p, DISCONTROL_LIMIT_X_P),

    // @Param: _LIMIT_Y_P
    // @DisplayName: distance y limit P gain
    // @Description: Less than how many centimeters for speed limit
    // @User: Standard
    // @units: cm
    // @Values: > 0
    AP_GROUPINFO("_LIMIT_Y_P",  11, AC_DistanceControl, _limit_y_p, DISCONTROL_LIMIT_Y_P),

    // @Param: _LIMIT_Z_P
    // @DisplayName: distance z limit P gain
    // @Description: Less than how many centimeters for speed limit
    // @User: Standard
    // @units: cm
    // @Values: > 0
    AP_GROUPINFO("_LIMIT_Z_P",  12, AC_DistanceControl, _limit_z_p, DISCONTROL_LIMIT_Z_P),

    // @Param: _THR_FACE_P
    // @DisplayName: face thrusts scale P gain
    // @Description: face thrusts scale P gain
    // @User: Standard
    // @Values: > 0
    AP_GROUPINFO("_THR_FACE_P",  13, AC_DistanceControl, _thr_face_p, DISCONTROL_THRUSTS_FACE_P),

    // @Param: _THR_LIMIT_P
    // @DisplayName: limit thrusts scale P gain
    // @Description: limit thrusts scale P gain
    // @User: Standard
    // @Values: > 0
    AP_GROUPINFO("_THR_LIMIT_P",  14, AC_DistanceControl, _thr_limit_p, DISCONTROL_THRUSTS_LIMIT_P),

    // @Param: _LIMIT_ENABLE
    // @DisplayName: Keep a distance and avoid collision
    // @Description: Keep a distance and avoid collision
    // @Values: 1:enable,0:disable
    // @User: Advanced
    AP_GROUPINFO("_LIMIT_ENABLE",  15, AC_DistanceControl, _limit_enable, 0),

	// @Param: _FRONT_LIMIT
    // @DisplayName: front distance limit 
    // @Description: front distance limit
    // @Unit: cm
    // @User: Advanced
    AP_GROUPINFO("_FRONT_LIMIT",  16, AC_DistanceControl, _front_limit_cm, DISCONTROL_FRONT_LIMIT_CM),

    // @Param: _BACK_LIMIT
    // @DisplayName: back distance limit 
    // @Description: back distance limit
    // @Unit: cm
    // @User: Advanced
    AP_GROUPINFO("_BACK_LIMIT",  17, AC_DistanceControl, _back_limit_cm, DISCONTROL_BACK_LIMIT_CM),

    // @Param: _LEFT_LIMIT
    // @DisplayName: left distance limit 
    // @Description: left distance limit
    // @Unit: cm
    // @User: Advanced
    AP_GROUPINFO("_LEFT_LIMIT",  18, AC_DistanceControl, _left_limit_cm, DISCONTROL_LEFT_LIMIT_CM),

    // @Param: _RIGHT_LIMIT
    // @DisplayName: right distance limit 
    // @Description: right distance limit
    // @Unit: cm
    // @User: Advanced
    AP_GROUPINFO("_RIGHT_LIMIT",  19, AC_DistanceControl, _right_limit_cm, DISCONTROL_RIGHT_LIMIT_CM),

    // @Param: _TOP_LIMIT
    // @DisplayName: top distance limit 
    // @Description: top distance limit
    // @Unit: cm
    // @User: Advanced
	AP_GROUPINFO("_TOP_LIMIT",  20, AC_DistanceControl, _top_limit_cm, DISCONTROL_TOP_LIMIT_CM),

    // @Param: _BOTTOM_LIMIT
    // @DisplayName: bottom distance limit 
    // @Description: bottom distance limit
    // @Unit: cm
    // @User: Advanced
	AP_GROUPINFO("_BOTTOM_LIMIT",  21, AC_DistanceControl, _bottom_limit_cm, DISCONTROL_BOTTOM_LIMIT_CM),

    // @Param: _FACE
    // @DisplayName: distance face
    // @Description: distance face
    // @Values: 1:front 2:back 4:left 8:right 16:top 32:bottom
    // @User: Advanced
    AP_GROUPINFO("_FACE",  30, AC_DistanceControl, _distance_face, 0),

	// @Param: _FRONT_OFT
    // @DisplayName: front offset
    // @Description: front offset
    // @Unit: cm
    // @User: Advanced
    AP_GROUPINFO("_FRONT_OFT",  31, AC_DistanceControl, _front_offset, DISCONTROL_FRONT_OFT_CM),

    // @Param: _BACK_OFT
    // @DisplayName: back offset
    // @Description: back offset
    // @Unit: cm
    // @User: Advanced
    AP_GROUPINFO("_BACK_OFT",  32, AC_DistanceControl, _back_offset, DISCONTROL_BACK_OFT_CM),

    // @Param: _LEFT_OFT
    // @DisplayName: left offset
    // @Description: left offset
    // @Unit: cm
    // @User: Advanced
    AP_GROUPINFO("_LEFT_OFT",  33, AC_DistanceControl, _left_offset, DISCONTROL_LEFT_OFT_CM),

    // @Param: _RIGHT_OFT
    // @DisplayName: right offset
    // @Description: right offset
    // @Unit: cm
    // @User: Advanced
    AP_GROUPINFO("_RIGHT_OFT",  34, AC_DistanceControl, _right_offset, DISCONTROL_RIGHT_OFT_CM),

    // @Param: _DELAY_X
    // @DisplayName: x delay time
    // @Description: x delay time
    // @User: Standard
    // @units: ms
    // @Values: > 0
    AP_GROUPINFO("_DELAY_X",  35, AC_DistanceControl, _delay_ms_x, DISCONTROL_DELAY_MS_X),

    // @Param: _DELAY_Y
    // @DisplayName: y delay time
    // @Description: y delay time
    // @User: Standard
    // @units: ms
    // @Values: > 0
    AP_GROUPINFO("_DELAY_Y",  36, AC_DistanceControl, _delay_ms_y, DISCONTROL_DELAY_MS_Y),

    // @Param: _DELAY_Z
    // @DisplayName: z delay time
    // @Description: z delay time
    // @User: Standard
    // @units: ms
    // @Values: > 0
    AP_GROUPINFO("_DELAY_Z",  37, AC_DistanceControl, _delay_ms_z, DISCONTROL_DELAY_MS_Z),

    // @Param: _ACCEL_X
    // @DisplayName: x max accel
    // @Description: x max accel
    // @User: Standard
    // @units: cm/s/s
    // @Values: > 0
    AP_GROUPINFO("_ACCEL_X",  38, AC_DistanceControl, _max_accel_x, DISCONTROL_MAX_ACCEL_X),

    // @Param: _ACCEL_Y
    // @DisplayName: y max accel
    // @Description: y max accel
    // @User: Standard
    // @units: cm/s/s
    // @Values: > 0
    AP_GROUPINFO("_ACCEL_Y",  39, AC_DistanceControl, _max_accel_y, DISCONTROL_MAX_ACCEL_Y),

    // @Param: _ACCEL_Z
    // @DisplayName: z max accel
    // @Description: z max accel
    // @User: Standard
    // @units: cm/s/s
    // @Values: > 0
    AP_GROUPINFO("_ACCEL_Z",  40, AC_DistanceControl, _max_accel_z, DISCONTROL_MAX_ACCEL_Z),

    // @Param: _CURVE_X
    // @DisplayName: x curve
    // @Description: x curve
    // @User: Standard
    // @Values: 1 2 3
    AP_GROUPINFO("_CURVE_X",  41, AC_DistanceControl, _curve_x, DISCONTROL_CURVE_X),

    // @Param: _CURVE_Y
    // @DisplayName: y curve
    // @Description: y curve
    // @User: Standard
    // @Values: 1 2 3
    AP_GROUPINFO("_CURVE_Y",  42, AC_DistanceControl, _curve_y, DISCONTROL_CURVE_Y),

    // @Param: _CURVE_Z
    // @DisplayName: z curve
    // @Description: z curve
    // @User: Standard
    // @Values: 1 2 3
    AP_GROUPINFO("_CURVE_Z",  43, AC_DistanceControl, _curve_z, DISCONTROL_CURVE_Z),

    // @Param: _SPEED_X
    // @DisplayName: x max speed
    // @Description: x max speed
    // @User: Standard
    // @units: cm/s
    // @Values: > 0
    AP_GROUPINFO("_SPEED_X",  44, AC_DistanceControl, _max_speed_x, DISCONTROL_MAX_SPEED_X),

    // @Param: _SPEED_Y
    // @DisplayName: y max speed
    // @Description: y max speed
    // @User: Standard
    // @units: cm/s
    // @Values: > 0
    AP_GROUPINFO("_SPEED_Y",  45, AC_DistanceControl, _max_speed_y, DISCONTROL_MAX_SPEED_Y),

    // @Param: _SPEED_Z
    // @DisplayName: z max speed
    // @Description: z max speed
    // @User: Standard
    // @units: cm/s
    // @Values: > 0
    AP_GROUPINFO("_SPEED_Z",  46, AC_DistanceControl, _max_speed_z, DISCONTROL_MAX_SPEED_Z),
    

    // @Param: _FRONT347_OFT
    // @DisplayName: front347 offset
    // @Description: front347 offset
    // @Unit: cm
    // @User: Advanced
    AP_GROUPINFO("_FRONT347_OFT",  47, AC_DistanceControl, _front347_offset, DISCONTROL_FRONT347_OFT_CM),

	// @Param: _FRONT13_OFT
    // @DisplayName: front13 offset
    // @Description: front13 offset
    // @Unit: cm
    // @User: Advanced
    AP_GROUPINFO("_FRONT13_OFT",  48, AC_DistanceControl, _front13_offset, DISCONTROL_FRONT13_OFT_CM),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_DistanceControl::AC_DistanceControl(const AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                             AP_Motors& motors, AC_AttitudeControl& attitude_control,
                             const RangeFinder& rangefinder) :
    _ahrs(ahrs),
    _inav(inav),
    _motors(motors),
    _attitude_control(attitude_control),
    _rangefinder(rangefinder),
    _p_pos_x(DISCONTROL_POS_X_P),
    _p_vel_x(DISCONTROL_VEL_X_P),
    _pid_accel_x(DISCONTROL_ACC_X_P, DISCONTROL_ACC_X_I, DISCONTROL_ACC_X_D, 0.0f, DISCONTROL_ACC_X_IMAX, 0.0f, DISCONTROL_ACC_X_FILT_HZ, 0.0f, DISCONTROL_ACC_X_DT),
	_p_pos_y(DISCONTROL_POS_Y_P),
    _p_vel_y(DISCONTROL_VEL_Y_P),
    _pid_accel_y(DISCONTROL_ACC_Y_P, DISCONTROL_ACC_Y_I, DISCONTROL_ACC_Y_D, 0.0f, DISCONTROL_ACC_Y_IMAX, 0.0f, DISCONTROL_ACC_Y_FILT_HZ, 0.0f, DISCONTROL_ACC_Y_DT),
    _p_pos_z(DISCONTROL_POS_Z_P),
    _p_vel_z(DISCONTROL_VEL_Z_P),
    _pid_accel_z(DISCONTROL_ACC_Z_P, DISCONTROL_ACC_Z_I, DISCONTROL_ACC_Z_D, 0.0f, DISCONTROL_ACC_Z_IMAX, 0.0f, DISCONTROL_ACC_Z_FILT_HZ, 0.0f, DISCONTROL_ACC_Z_DT),
    _dt(DISCONTROL_DT_400HZ),
    _vel_x_error_filter(DISCONTROL_VEL_X_ERROR_CUTOFF_FREQ),
    _vel_y_error_filter(DISCONTROL_VEL_Y_ERROR_CUTOFF_FREQ),
    _vel_z_error_filter(DISCONTROL_VEL_Z_ERROR_CUTOFF_FREQ),
    _out_x_filter(DISCONTROL_OUT_X_CUTOFF_FREQ),
    _out_y_filter(DISCONTROL_OUT_Y_CUTOFF_FREQ)
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

    _singleton = this;
}

void AC_DistanceControl::update_distance(void)
{
	distance_bf[DIS_BF_FRONT] = _rangefinder.distance_cm_orient(ROTATION_NONE);
	distance_bf[DIS_BF_BACK] = _rangefinder.distance_cm_orient(ROTATION_PITCH_180);
	distance_bf[DIS_BF_LEFT] = _rangefinder.distance_cm_orient(ROTATION_YAW_270);
	distance_bf[DIS_BF_RIGHT] = _rangefinder.distance_cm_orient(ROTATION_YAW_90);
	distance_bf[DIS_BF_TOP] = _rangefinder.distance_cm_orient(ROTATION_PITCH_90);
	distance_bf[DIS_BF_BOTTOM] = _rangefinder.distance_cm_orient(ROTATION_PITCH_270);
	distance_bf[DIS_BF_FRONT347] = _rangefinder.distance_cm_orient(ROTATION_YAW_315);
    distance_bf[DIS_BF_FRONT13] = _rangefinder.distance_cm_orient(ROTATION_YAW_45);

	if(distance_bf[DIS_BF_FRONT] >  _front_offset)
		distance_bf[DIS_BF_FRONT] -= _front_offset;
	else if(distance_bf[DIS_BF_FRONT] ==  _front_offset)
		distance_bf[DIS_BF_FRONT] = 0;
	else 
		distance_bf[DIS_BF_FRONT] = 0;
		
	if(distance_bf[DIS_BF_BACK] > _back_offset)
		distance_bf[DIS_BF_BACK] -= _back_offset;
	else if(distance_bf[DIS_BF_BACK] == _back_offset)
		distance_bf[DIS_BF_BACK] = 0;
	else 
		distance_bf[DIS_BF_BACK] = 0;
		
	if(distance_bf[DIS_BF_RIGHT] >  _right_offset)
		distance_bf[DIS_BF_RIGHT] -= _right_offset;
	else if(distance_bf[DIS_BF_RIGHT] ==  _right_offset)
		distance_bf[DIS_BF_RIGHT] = 0;
	else 
		distance_bf[DIS_BF_RIGHT] = 0;
		
	if(distance_bf[DIS_BF_LEFT] > _left_offset)
		distance_bf[DIS_BF_LEFT] -= _left_offset;
	else if(distance_bf[DIS_BF_LEFT] == _left_offset)
		distance_bf[DIS_BF_LEFT] = 0;
	else 
		distance_bf[DIS_BF_LEFT] = 0;

	if(distance_bf[DIS_BF_FRONT347] >  _front347_offset)
		distance_bf[DIS_BF_FRONT347] -= _front347_offset;
	else if(distance_bf[DIS_BF_FRONT347] ==  _front347_offset)
		distance_bf[DIS_BF_FRONT347] = 0;
	else 
		distance_bf[DIS_BF_FRONT347] = 0;

	if(distance_bf[DIS_BF_FRONT13] >  _front13_offset)
		distance_bf[DIS_BF_FRONT13] -= _front13_offset;
	else if(distance_bf[DIS_BF_FRONT13] ==  _front13_offset)
		distance_bf[DIS_BF_FRONT13] = 0;
	else 
		distance_bf[DIS_BF_FRONT13] = 0;

	if(distance_bf[DIS_BF_FRONT347] != 0 || distance_bf[DIS_BF_FRONT13] != 0) {
		if(distance_bf[DIS_BF_FRONT347]==0)
			distance_bf[DIS_BF_FRONT] = distance_bf[DIS_BF_FRONT13]*cosf(radians(13));
		else if(distance_bf[DIS_BF_FRONT13]==0)
			distance_bf[DIS_BF_FRONT] = distance_bf[DIS_BF_FRONT347]*cosf(radians(13));
		else
			distance_bf[DIS_BF_FRONT] = MIN(distance_bf[DIS_BF_FRONT347]*cosf(radians(13)), distance_bf[DIS_BF_FRONT13]*cosf(radians(13)));
	}
	
	Matrix3f m;
	m.from_euler(_ahrs.roll, _ahrs.pitch, 0);

	Vector3f dist[DISTANCE_NUM] = {{(float)distance_bf[DIS_BF_FRONT], 0, 0},
						{(float)-distance_bf[DIS_BF_BACK], 0, 0},
						{0, (float)-distance_bf[DIS_BF_LEFT], 0},
						{0, (float)distance_bf[DIS_BF_RIGHT], 0},
						{0, 0, (float)-distance_bf[DIS_BF_TOP]},
						{0, 0, (float)distance_bf[DIS_BF_BOTTOM]}};
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

	//filter
	for(int i=0; i<DISTANCE_NUM; i++) {
		if(abs(distance_ned[i]) < 5)
			distance_ned[i] = 0;
	}
	
	if(1) {
		static uint32_t _startup_ms = 0;

		if(_startup_ms == 0) {
			_startup_ms = AP_HAL::millis();
		}

		if(AP_HAL::millis() - _startup_ms > 1000) {
			_startup_ms = AP_HAL::millis();

			/*for(int i=0; i<DISTANCE_NUM; i++) {
				hal.shell->printf("[%.2f %.2f %.2f]\r\n", 
							dist[i].x,
							dist[i].y, 
							dist[i].z);
			}*/
			
			hal.shell->printf("\r\ndistance_bf: %d %d %d %d %d %d %d %d\r\n", 
						distance_bf[DIS_BF_FRONT],
						distance_bf[DIS_BF_BACK], 
						distance_bf[DIS_BF_LEFT], 
						distance_bf[DIS_BF_RIGHT],
						distance_bf[DIS_BF_TOP],
						distance_bf[DIS_BF_BOTTOM],
						distance_bf[DIS_BF_FRONT347],
						distance_bf[DIS_BF_FRONT13]);

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

void AC_DistanceControl::pilot_thrusts_scale(Vector3f &thrusts)
{
	if(_distance_face) {
		thrusts *= _thr_face_p;
	} else if(_limit_enable) {
		thrusts *= _thr_limit_p;
	}
}

void AC_DistanceControl::pilot_thrusts_limit(Vector3f &thrusts)
{
	Vector3f dis_error;
	
    if(thrusts.x > 0.0f) {
    	if(_front_limit_cm != 0 && distance_ned[DISTANCE_FRONT] != 0) {
			dis_error.x = distance_ned[DISTANCE_FRONT] - _front_limit_cm;
		} else {
			dis_error.x = _limit_x_p;
		}
	} else if(thrusts.x < -0.0f) {
		if(_back_limit_cm != 0 && distance_ned[DISTANCE_BACK] != 0) {
			dis_error.x = _back_limit_cm - distance_ned[DISTANCE_BACK];
		} else {
			dis_error.x = _limit_x_p;
		}
	} else {
		dis_error.x = 0;
		thrusts.x = 0;
	}
	thrusts.x *= constrain_float(powf(dis_error.x/_limit_x_p, _curve_x), 0.0f, 1.0f);

	if(thrusts.y > 0.0f) {
		if(_right_limit_cm != 0 && distance_ned[DISTANCE_RIGHT] != 0) {
			dis_error.y = distance_ned[DISTANCE_RIGHT] - _right_limit_cm;
		} else {
			dis_error.y = _limit_y_p;
		}
	} else if(thrusts.y < -0.0f) {
		if(_left_limit_cm != 0 && distance_ned[DISTANCE_LEFT] != 0) {
			dis_error.y = _left_limit_cm - distance_ned[DISTANCE_LEFT];
		} else {
			dis_error.y = _limit_y_p;
		}
	} else {
		dis_error.y = 0;
		thrusts.y = 0;
	}
	thrusts.y *= constrain_float(powf(dis_error.y/_limit_y_p, _curve_y), 0.0f, 1.0f);

	if(thrusts.z < -0.0f) {
		if(_bottom_limit_cm != 0 && distance_ned[DISTANCE_BOTTOM] != 0) {
			dis_error.z = distance_ned[DISTANCE_BOTTOM] - _bottom_limit_cm;
		} else {
			dis_error.z = _limit_z_p;
		}
	} else if(thrusts.z > 0.0f) {
		if(abs(distance_ned[DISTANCE_TOP]) > 10 && _top_limit_cm != 0)
			dis_error.z = _top_limit_cm - distance_ned[DISTANCE_TOP];
		else
			dis_error.z = _limit_z_p;
	} else {
		dis_error.z = 0;
		thrusts.z = 0;
	}
	thrusts.z *= constrain_float(powf(dis_error.z/_limit_z_p, _curve_z), 0.0f, 1.0f);

	if(1) {
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

			hal.shell->printf("thrusts [%.4f %.4f %.4f]\r\n",
					thrusts.x, 
					thrusts.y,
					thrusts.z); 
		}
	}
}

void AC_DistanceControl::attitude_filter(Vector3f &thrusts)
{
	int32_t roll = _ahrs.roll_sensor/100;
	int32_t pitch = _ahrs.pitch_sensor/100;
	//int32_t yaw = _ahrs.yaw_sensor/100;
	
	if(_limit_enable) {
		if(pitch > 5) {
			distance_ned[DISTANCE_BACK] = 0;
		} else if(pitch < -5) {
			distance_ned[DISTANCE_FRONT] = 0;
		}

		if(roll > 5) {
			distance_ned[DISTANCE_RIGHT] = 0;
		} else if(roll < -5) {
			distance_ned[DISTANCE_LEFT] = 0;
		}
	}
}

void AC_DistanceControl::update_backend(Vector3f &thrusts)
{
	update_distance();

	attitude_filter(thrusts);

	pilot_thrusts_scale(thrusts);

	if(_limit_enable) {
		pilot_thrusts_limit(thrusts);
	}
}

void AC_DistanceControl::relax_z_controller(float distance)
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

void AC_DistanceControl::update_z_controller(float distance)
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
		hal.shell->printf("pt.z[%.4f] pc.z[%.4f] pe.z[%.4f]\r\n", 
					_pos_target.z,
					curr_alt,
					_pos_error.z);
	}

    // calculate _vel_target.z using from _pos_error.z using sqrt controller
    _vel_target.z = AC_AttitudeControl::sqrt_controller(_pos_error.z, _p_pos_z.kP(), _max_accel_z, _dt);

	if(_max_speed_z > 0) {
	    if (_vel_target.z > _max_speed_z) {
	        _vel_target.z = _max_speed_z;
	    }

	    if (_vel_target.z < -_max_speed_z) {
	        _vel_target.z = -_max_speed_z;
	    }
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
        _vel_z_error_filter.reset(0);
        _flags.reset_rate_to_accel_z = false;
    } else {
        // calculate rate error and filter with cut off frequency of 2 Hz
        _vel_error.z = _vel_z_error_filter.apply(_vel_target.z - curr_vel.z, _dt);
    }

	if(print_flag) {
		hal.shell->printf("vt.z[%.4f] vc.z[%.4f] ve.z[%.4f]\r\n", 
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
		hal.shell->printf("at.z[%.4f] ac.z[%.4f] ae.z[%.4f] pout.z[%.4f] hover[%.4f] out.z[%.4f]\r\n", 
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

void AC_DistanceControl::relax_x_controller(float distance)
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

void AC_DistanceControl::update_x_controller(float distance)
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
		hal.shell->printf("pt.x[%.4f] pc.x[%.4f] pe.x[%.4f]\r\n", 
					_pos_target.x,
					curr_dis,
					_pos_error.x);
	}

    // calculate _vel_target.x using from _pos_error.x using sqrt controller
    _vel_target.x = AC_AttitudeControl::sqrt_controller(_pos_error.x, _p_pos_x.kP(), _max_accel_x, _dt);

	if(_max_speed_x > 0) {
	    if (_vel_target.x > _max_speed_x) {
	        _vel_target.x = _max_speed_x;
	    }

	    if (_vel_target.x < -_max_speed_x) {
	        _vel_target.x = -_max_speed_x;
	    }
    }

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
		hal.shell->printf("vt.x[%.4f] vc.x[%.4f] ve.x[%.4f]\r\n", 
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
		hal.shell->printf("at.x[%.4f] ac.x[%.4f] ae.x[%.4f] pout.x[%.4f]\r\n", 
					_accel_target.x,
					x_accel_meas,
					_accel_target.x - x_accel_meas,
					pid_out);
	}

	out_filtered = _out_x_filter.apply(pid_out, _dt);
    _motors.set_forward(out_filtered);
    
    print_flag = 0;
}

void AC_DistanceControl::relax_y_controller(float distance)
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

void AC_DistanceControl::update_y_controller(float distance)
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
		hal.shell->printf("pt.y[%.4f] pc.y[%.4f] pe.y[%.4f]\r\n", 
					_pos_target.y,
					curr_dis,
					_pos_error.y);
	}

    // calculate _vel_target.y using from _pos_error.y using sqrt controller
    _vel_target.y = AC_AttitudeControl::sqrt_controller(_pos_error.y, _p_pos_y.kP(), _max_accel_y, _dt);

	if(_max_speed_y > 0) {
	    if (_vel_target.y > _max_speed_y) {
	        _vel_target.y = _max_speed_y;
	    }

	    if (_vel_target.y < -_max_speed_y) {
	        _vel_target.y = -_max_speed_y;
	    }
    }

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
		hal.shell->printf("vt.y[%.4f] vc.y[%.4f] ve.y[%.4f]\r\n", 
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
		hal.shell->printf("at.y[%.4f] ac.y[%.4f] ae.y[%.4f] pout.y[%.4f]\r\n", 
					_accel_target.y,
					y_accel_meas,
					_accel_target.y - y_accel_meas,
					pid_out);
	}

	out_filtered = _out_y_filter.apply(pid_out, _dt);
    _motors.set_lateral(out_filtered);
    
    print_flag = 0;
}

AC_DistanceControl *AC_DistanceControl::_singleton;

namespace AP {

AC_DistanceControl *distance_control()
{
    return AC_DistanceControl::get_singleton();
}

}


