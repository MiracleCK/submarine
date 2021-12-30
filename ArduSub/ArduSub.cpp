/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// ArduSub scheduling, originally copied from ArduCopter

#include "Sub.h"
#include <AP_HAL_ChibiOS/RCOutput.h>

extern void param_debug_tick(void);

#define SCHED_TASK(func, rate_hz, max_time_micros) SCHED_TASK_CLASS(Sub, &sub, func, rate_hz, max_time_micros)

/*
  scheduler table for fast CPUs - all regular tasks apart from the fast_loop()
  should be listed here, along with how often they should be called (in hz)
  and the maximum time they are expected to take (in microseconds)
 */
const AP_Scheduler::Task Sub::scheduler_tasks[] = {
    SCHED_TASK(fifty_hz_loop,         50,     75),
    SCHED_TASK(update_GPS,            50,    200),
#if OPTFLOW == ENABLED
    SCHED_TASK_CLASS(OpticalFlow,          &sub.optflow,             update,         200, 160),
#endif
    SCHED_TASK(update_batt_compass,   10,    120),
    SCHED_TASK(read_rangefinder,      20,    100),
    SCHED_TASK(update_altitude,       10,    100),
    SCHED_TASK(three_hz_loop,          3,     75),
    SCHED_TASK(update_turn_counter,   10,     50),
    SCHED_TASK_CLASS(AP_Baro,             &sub.barometer,    accumulate,          50,  90),
    SCHED_TASK_CLASS(AP_Notify,           &sub.notify,       update,              50,  90),
    SCHED_TASK(one_hz_loop,            1,    100),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_receive,     400, 180),
    SCHED_TASK_CLASS(GCS,                 (GCS*)&sub._gcs,   update_send,        400, 550),
#if AC_FENCE == ENABLED
    SCHED_TASK_CLASS(AC_Fence,            &sub.fence,        update,              10, 100),
#endif
#if MOUNT == ENABLED
    SCHED_TASK_CLASS(AP_Mount,            &sub.camera_mount, update,              50,  75),
#endif
#if CAMERA == ENABLED
    SCHED_TASK_CLASS(AP_Camera,           &sub.camera,       update_trigger,      50,  75),
#endif
    SCHED_TASK(ten_hz_logging_loop,   10,    350),
    SCHED_TASK(twentyfive_hz_logging, 25,    110),
    SCHED_TASK_CLASS(AP_Logger,     &sub.logger,    periodic_tasks,     400, 300),
    SCHED_TASK_CLASS(AP_InertialSensor,   &sub.ins,          periodic,           400,  50),
    SCHED_TASK_CLASS(AP_Scheduler,        &sub.scheduler,    update_logging,     0.1,  75),
#if RPM_ENABLED == ENABLED
    SCHED_TASK(rpm_update,            10,    200),
#endif
    SCHED_TASK_CLASS(Compass,          &sub.compass,              cal_update, 100, 100),
    SCHED_TASK(accel_cal_update,      10,    100),
    SCHED_TASK(terrain_update,        10,    100),
#if GRIPPER_ENABLED == ENABLED
    SCHED_TASK_CLASS(AP_Gripper,          &sub.g2.gripper,       update,              10,  75),
#endif
#ifdef USERHOOK_FASTLOOP
    SCHED_TASK(userhook_FastLoop,    100,     75),
#endif
#ifdef USERHOOK_50HZLOOP
    SCHED_TASK(userhook_50Hz,         50,     75),
#endif
#ifdef USERHOOK_MEDIUMLOOP
    SCHED_TASK(userhook_MediumLoop,   10,     75),
#endif
#ifdef USERHOOK_SLOWLOOP
    SCHED_TASK(userhook_SlowLoop,     3.3,    75),
#endif
#ifdef USERHOOK_SUPERSLOWLOOP
    SCHED_TASK(userhook_SuperSlowLoop, 1,   75),
#endif
};

constexpr int8_t Sub::_failsafe_priorities[5];

void Sub::setup()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
	factory.test_check();
#endif

    // Load the default values of variables listed in var_info[]s
    AP_Param::setup_sketch_defaults();

    init_ardupilot();
    control_mode = (control_mode_t)g.cr_default_mode.get();

    // initialise the main loop scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks), MASK_LOG_PM);
    reset();
}

void Sub::loop()
{
    scheduler.loop();
    G_Dt = scheduler.get_loop_period_s();
}

void Sub::reset()
{
    if (control_mode == MANUAL)
        manual_init();
    else
        wash_init(control_mode);
    hal.shell->printf("Reset\r\n");
}

// Main loop - 400hz
void Sub::fast_loop()
{
    // update INS immediately to get current gyro data populated
    ins.update();

    //don't run rate controller in manual or motordetection modes
    if (control_mode != MANUAL && control_mode != MOTOR_DETECT && control_mode < ULTRA) {
        // run low level rate controllers that only require IMU data
        attitude_control.rate_controller_run();
    }

    // send outputs to the motors library
    motors_output();

    // run EKF state estimator (expensive)
    // --------------------
    read_AHRS();

    // Inertial Nav
    // --------------------
    read_inertia();

    // check if ekf has reset target heading
    check_ekf_yaw_reset();

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
	if(!factory.isFactoryTestMode()) 
#endif
	{
	    //control_lateral();
	    // run the attitude controllers
	    update_flight_mode();
    }

    // update home from EKF if necessary
    update_home_from_EKF();

    // check if we've reached the surface or bottom
    update_surface_and_bottom_detector();

#if MOUNT == ENABLED
    // camera mount's fast update
    camera_mount.update_fast();
#endif

    // log sensor health
    if (should_log(MASK_LOG_ANY)) {
        Log_Sensor_Health();
    }

    param_debug_tick();

    //factory.aging_check_reboot();
}

// 50 Hz tasks
void Sub::fifty_hz_loop()
{
    // check pilot input failsafe
    failsafe_pilot_input_check();

    failsafe_crash_check();

    failsafe_ekf_check();

    failsafe_sensors_check();

    // Update rc input/output
    rc().read_input();
    SRV_Channels::output_ch_all();
}

// update_batt_compass - read battery and compass
// should be called at 10hz
void Sub::update_batt_compass()
{
    // read battery before compass because it may be used for motor interference compensation
    battery.read();

    if (AP::compass().enabled()) {
        // update compass with throttle value - used for compassmot
        compass.set_throttle(motors.get_throttle());
        compass.read();
    }
}

// ten_hz_logging_loop
// should be run at 10hz
void Sub::ten_hz_logging_loop()
{
    // log attitude data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        logger.Write_Rate(&ahrs_view, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.get_accel_z_pid().get_pid_info());
        }
    }
    if (should_log(MASK_LOG_MOTBATT)) {
        Log_Write_MotBatt();
    }
    if (should_log(MASK_LOG_RCIN)) {
        logger.Write_RCIN();
    }
    if (should_log(MASK_LOG_RCOUT)) {
        logger.Write_RCOUT();
    }
    if (should_log(MASK_LOG_NTUN) && mode_requires_GPS(control_mode)) {
        pos_control.write_log();
    }
    if (should_log(MASK_LOG_IMU) || should_log(MASK_LOG_IMU_FAST) || should_log(MASK_LOG_IMU_RAW)) {
        logger.Write_Vibration();
    }
    if (should_log(MASK_LOG_CTUN)) {
        attitude_control.control_monitor_log();
    }
}

// twentyfive_hz_logging_loop
// should be run at 25hz
void Sub::twentyfive_hz_logging()
{
    smoothly_control_pump();
    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
        logger.Write_Rate(&ahrs_view, motors, attitude_control, pos_control);
        if (should_log(MASK_LOG_PID)) {
            logger.Write_PID(LOG_PIDR_MSG, attitude_control.get_rate_roll_pid().get_pid_info());
            logger.Write_PID(LOG_PIDP_MSG, attitude_control.get_rate_pitch_pid().get_pid_info());
            logger.Write_PID(LOG_PIDY_MSG, attitude_control.get_rate_yaw_pid().get_pid_info());
            logger.Write_PID(LOG_PIDA_MSG, pos_control.get_accel_z_pid().get_pid_info());
        }
    }

    // log IMU data if we're not already logging at the higher rate
    if (should_log(MASK_LOG_IMU) && !should_log(MASK_LOG_IMU_RAW)) {
        logger.Write_IMU();
    }
}

// three_hz_loop - 3.3hz loop
void Sub::three_hz_loop()
{
    leak_detector.update();
    water_detector->update();

    if (water_detector->read() == 3)
    {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Out of water");
    }

    failsafe_leak_check();

    failsafe_internal_pressure_check();

    failsafe_internal_temperature_check();

    // check if we've lost contact with the ground station
    failsafe_gcs_check();

    // check if we've lost terrain data
    failsafe_terrain_check();

#if AC_FENCE == ENABLED
    // check if we have breached a fence
    fence_check();
#endif // AC_FENCE_ENABLED

    ServoRelayEvents.update_events();
}

// one_hz_loop - runs at 1Hz
void Sub::one_hz_loop()
{
    bool arm_check = arming.pre_arm_checks(false);
    ap.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_check = arm_check;
    AP_Notify::flags.pre_arm_gps_check = position_ok();
    AP_Notify::flags.flying = motors.armed();

    if (should_log(MASK_LOG_ANY)) {
        Log_Write_Data(LogDataID::AP_STATE, ap.value);
    }

    /*if (!motors.armed()) {
        // make it possible to change ahrs orientation at runtime during initial config
        ahrs.update_orientation();

        // set all throttle channel settings
        motors.set_throttle_range(-2200, 2200);
    }*/
    update_neo_led();

    // update assigned functions and enable auxiliary servos
    SRV_Channels::enable_aux_servos();

    // update position controller alt limits
    update_poscon_alt_max();

    // log terrain data
    terrain_logging();

    // need to set "likely flying" when armed to allow for compass
    // learning to run
    ahrs.set_likely_flying(hal.util->get_soft_armed());

    const Matrix3f &m = ahrs.get_rotation_body_to_ned();
    int16_t left, right;
    uint16_t lp, rp, p;
    uint8_t red, green, blue;
    SRV_Channels::get_output_pwm(SRV_Channel::k_motor1, (uint16_t &)left);
    SRV_Channels::get_output_pwm(SRV_Channel::k_motor2, (uint16_t &)right);
    SRV_Channels::get_output_pwm(SRV_Channel::k_throttleLeft, lp);
    SRV_Channels::get_output_pwm(SRV_Channel::k_throttleRight, rp);
    SRV_Channels::get_output_pwm(SRV_Channel::k_boost_throttle, p);

    int16_t comm = left + right;
    int16_t diff = left - right;
    green = (uint8_t)((abs(comm))*255/8000);
    if (diff >= 0)
    {
        blue = (uint8_t)(diff*255/8000);
        red = 0;
    }
    else
    {
        red = (uint8_t)(-diff*255/8000);
        blue = 0;
    }

    hal.rcout->set_neopixel_rgb_data(6, 2, red, green, blue);
    hal.rcout->neopixel_send();

    int32_t rpm[6];
    int32_t cnt = STM32_TIM9->CNT;
    if (cnt)
        STM32_TIM9->CNT = 0;
    rpm[0] = cnt;
    cnt = STM32_TIM8->CNT;
    if (cnt)
        STM32_TIM8->CNT = 0;
    rpm[1] = cnt;

#if BIDIR_DSHOT
    ChibiOS::RCOutput * rco = (ChibiOS::RCOutput *)hal.rcout;
    rco->get_dshot_telemetry(rpm + 2);
#else
    rpm[2] = 0;
    rpm[3] = 0;
    rpm[4] = 0;
    rpm[5] = 0;
#endif
    char buf[100];
    snprintf(buf, 100, "RPM %d %d %d %d %d %d", (int)rpm[0], (int)rpm[1], (int)rpm[2], (int)rpm[3], (int)rpm[4], (int)rpm[5]);
    printf("%s\r\n", buf);
    gcs().send_text(MAV_SEVERITY_INFO, buf);

    /*const Vector3f &mag = ahrs.get_compass()->get_field();
    const Vector3f &gyr = ahrs.get_gyro_latest();
    hal.shell->printf("%.1f %.1f %.1f %.5f %.5f %.5f\r\n",
                      mag.x, mag.y, mag.z,
                      gyr.x, gyr.y, gyr.z);*/


    /*uint32_t rx = hal.uartD->available();

    if (rx)
    {
        hal.shell->printf("N:%d ", rx);
        for (int i = 0; i < rx; ++i)
        {
            hal.shell->printf("%02X", hal.uartD->read());
        }
        hal.shell->printf("\r\n");
    }*/
}

// called at 50hz
void Sub::update_GPS()
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];   // time of last gps message
    bool gps_updated = false;

    gps.update();

    // log after every gps message
    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            gps_updated = true;
            break;
        }
    }

    if (gps_updated) {
#if CAMERA == ENABLED
        camera.update();
#endif
    }
}

void Sub::read_AHRS()
{
    // Perform IMU calculations and get attitude info
    //-----------------------------------------------
    // <true> tells AHRS to skip INS update as we have already done it in fast_loop()
    ahrs.update(true);
    ahrs_view.update(true);

    motors.set_roll_pitch_thr(ahrs.get_roll(), ahrs.get_pitch());
}

// read baro and rangefinder altitude at 10hz
void Sub::update_altitude()
{
    // read in baro altitude
    read_barometer();

    if (should_log(MASK_LOG_CTUN)) {
        Log_Write_Control_Tuning();
    }
}

bool Sub::control_check_barometer()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if (!ap.depth_sensor_present) { // can't hold depth without a depth sensor
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor is not connected.");
        return false;
    } else if (failsafe.sensor_health) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Depth sensor error.");
        return false;
    }
#endif
    return true;
}

void Sub::update_neo_led(void)
{
    uint32_t ns;
    if (motors.armed())
    {
        if (_status == PAUSE)
        {
            switch (water_detector->read())
            {
                case 0:
                    ns = NEO_LED(NEO_PATTERN_BLINK, NEO_GREEN);
                    break;
                case 1:
                    ns = NEO_LED(NEO_PATTERN_BLINK, NEO_PINK);
                    break;
                case 2:
                    ns = NEO_LED(NEO_PATTERN_BLINK, NEO_BROWN);
                    break;
                default:
                    ns = NEO_LED(NEO_PATTERN_BLINK, NEO_PURPLE);
                    break;
            }
        }
        else if (_status == ERROR)
            ns = NEO_LED(NEO_PATTERN_BLINK, NEO_PURPLE);
        else
            ns = NEO_LED(NEO_PATTERN_LOOP, NEO_GREEN);
    }
    else
    {
        switch (water_detector->read())
        {
            case 0:
                ns = NEO_LED(NEO_PATTERN_KEEP, NEO_YELLOW);
                break;
            case 1:
                ns = NEO_LED(NEO_PATTERN_KEEP, NEO_PINK);
                break;
            case 2:
                ns = NEO_LED(NEO_PATTERN_KEEP, NEO_BROWN);
                break;
            default:
                ns = NEO_LED(NEO_PATTERN_KEEP, NEO_PURPLE);
                break;
        }
    }

    uint32_t pattern = (ns>>24)&7;
    if (ns == (neo_led_state&0xFFFFFFF))
    {
        if (pattern == NEO_PATTERN_KEEP)
            return;
    }
    else
    {
        neo_led_state = ns;
    }
    uint8_t *state = (uint8_t *)&neo_led_state;

    uint32_t led_mask, except, step;
    switch (pattern)
    {
        //set
        case NEO_PATTERN_KEEP:
            hal.rcout->set_neopixel_rgb_data(6, 0xF,
                    state[2], state[1], state[0]);
            state[3] = NEO_PATTERN_KEEP;
            break;
        //blink
        case NEO_PATTERN_BLINK:
            if (state[3]&0x10)
            {
                state[3] = NEO_PATTERN_BLINK;
                hal.rcout->set_neopixel_rgb_data(
                        6, 0xF,
                        0, 0, 0);
            }
            else
            {
                state[3] = (0x10|NEO_PATTERN_BLINK);
                hal.rcout->set_neopixel_rgb_data(
                        6, 0xF,
                        state[2], state[1], state[0]);
            }
            break;
        //blink on
        case NEO_PATTERN_BLINK1:
        case NEO_PATTERN_BLINK2:
        case NEO_PATTERN_BLINK3:
        case NEO_PATTERN_BLINK4:
            if (state[3]&0x10)
            {
                state[3] = pattern;
                except = 1<<(pattern - NEO_PATTERN_BLINK1);
                led_mask = 0xF&(~except);
                hal.rcout->set_neopixel_rgb_data(
                        6, led_mask,
                        0, 0, 0);
            }
            else
            {
                state[3] = (0x10|pattern);
                hal.rcout->set_neopixel_rgb_data(
                        6, 0xF,
                        state[2], state[1], state[0]);
            }
            break;
        //loop
        case NEO_PATTERN_LOOP:
            step = state[3]>>4;
            if (step > 2)
            {
                state[3] = NEO_PATTERN_LOOP;
                hal.rcout->set_neopixel_rgb_data(6, 8,
                        state[2], state[1], state[0]);
                hal.rcout->set_neopixel_rgb_data(6, 7, 0, 0, 0);
            }
            else
            {
                uint32_t n = 1<<step;
                hal.rcout->set_neopixel_rgb_data(6, n,
                        state[2], state[1], state[0]);
                hal.rcout->set_neopixel_rgb_data(6, (0xF^n), 0, 0, 0);
                state[3] = NEO_PATTERN_LOOP|(step + 1)<<4;
            }
            break;
        default:
            return;
    }

    hal.rcout->neopixel_send();
}

AP_HAL_MAIN_CALLBACKS(&sub);
