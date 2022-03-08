#include "Sub.h"

#define SQRT3_2 0.866f
bool Sub::wash_init(control_mode_t mode)
{
    target_direction.zero();
    _now = AP_HAL::millis();
    mode_sum_ms = 0;
    mode_active_ts = 0;
    lateral_control.num = 0;
    if (mode == PULLUP)
    {
        mode_max_ms = 120000; //2min
    }
    else
    {
        switch (mode)
        {
            case FLOOR:
                mode_max_ms = 1800000; //30min
                break;
            case WATERLINE:
                mode_max_ms = 1800000; //30min
                break;
            case SMART:
                mode_max_ms = 1800000; //30min
                break;
            case REGULAR:
                mode_max_ms = 3600000; //60min
                break;
            case FAST:
                mode_max_ms = 2400000; //40min
                break;
            case ULTRA:
                mode_max_ms = 7200000; //120min
                break;
            default:
                mode_max_ms = 0;
                break;
        }
    }
    set_status(PENDING);
    _phase = WALL_RIGHT;
    return true;
}

void Sub::wash_run(void)
{
    _now = AP_HAL::millis();

    const Matrix3f &m = ahrs.get_rotation_body_to_ned();
    v_forward.x = m.a.x;
    v_forward.y = m.b.x;
    v_forward.z = m.c.x;
    v_downward.x = m.a.z;
    v_downward.y = m.b.z;
    v_downward.z = m.c.z;

    if (!motors.armed())
    {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0, true, g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        set_pump(IDLE);
        mode_active_ts = 0;
        if (sub_errors == 0 && !is_zero(channel_forward->norm_input())) {
            arming.arm(AP_Arming::Method::AUXSWITCH, false);
        }
        _status_ts = _now;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (_status == PAUSE)
    {
        mode_active_ts = 0;
        float fwd = channel_forward->norm_input();
        float yaw = channel_yaw->norm_input();
        bool surface = (water_detector->read() & 1) != 0;
        pump3 = channel_left_pump->get_radio_in();
        pump4 = channel_right_pump->get_radio_in();
        if (_is_manual_pump)
        {
            pump1 = channel_pump1->get_radio_in();
            pump2 = channel_pump2->get_radio_in();
            motors.set_forward(fwd);
            motors.set_yaw(yaw);
        }
        else
        {
            if (channel_pump1->get_radio_in() != 1500)
            {
                pump1 = channel_pump1->get_radio_in();
                _is_manual_pump = true;
            }

            if (channel_pump2->get_radio_in() != 1500)
            {
                pump2 = channel_pump2->get_radio_in();
                _is_manual_pump = true;
            }
            if (is_on_wall(v_downward))
            {
                if (surface)
                {
                    _step_ts = _now;
                }
                else if (_step_ts != 0)
                {
                    uint32_t dt = _now - _step_ts;
                    if (dt > 3000)
                    {
                        _step_ts = 0;
                    }
                }

                if (_step_ts)
                {
                    if (is_negative(fwd))
                    {
                        pump1 = 1560;
                        pump2 = 1560;
                        motors.set_forward(fwd);
                        motors.set_yaw(yaw);
                    }
                    else
                    {
                        motors.set_yaw(0);
                        if (is_positive(yaw))
                        {
                            pump1 = 1500 + yaw*350;
                            pump2 = 1560;
                            motors.set_forward(1.0f);
                        }
                        else if(is_negative(yaw))
                        {
                            pump1 = 1560;
                            pump2 = 1500 - yaw*350;
                            motors.set_forward(1.0f);
                        }
                        else
                        {
                            pump1 = 1800;
                            pump2 = 1800;
                            motors.set_forward(fwd);
                        }
                    }
                }
                else
                {
                    pump1 = 1800;
                    pump2 = 1800;
                    motors.set_forward(fwd);
                    motors.set_yaw(yaw);
                }
            }
            else
            {
                pump1 = 1750;
                pump2 = 1750;
                motors.set_forward(fwd);
                motors.set_yaw(yaw);
            }
        }
        return;
    }
    else {
        if (!is_zero(channel_forward->norm_input())) {
            set_status(PAUSE);
        }
    }

    if (mode_active_ts > 0)
    {
        uint32_t dt = _now - mode_active_ts;
        mode_sum_ms += dt;
    }

    if (mode_sum_ms >= mode_max_ms)
    {
        arming.disarm();
        mode_sum_ms = mode_max_ms;
        gcs().send_text(MAV_SEVERITY_INFO, "Task is done");
        if (control_mode == PULLUP)
        {
            set_mode(prev_control_mode, ModeReason::TERMINATE);
        }
        return;
    }
    mode_active_ts = _now;

    if (control_mode != FLOOR)
    {
        if (_phase == WALL_RIGHT && mode_sum_ms > mode_max_ms/2)
            _phase = BOTTOM;
    }

    switch (_status)
    {
        case PENDING:
        {
            if (_now - _step_ts < 2000)
            {
                if (_pump != NORMAL)
                    set_pump(NORMAL);
                break;
            }
            float cos = v_forward*target_direction;
            if (cos < COS_10)
            {
                //not stable, keep waiting
                _step_ts = _now;
                target_direction = v_forward;
                break;
            }

            //Being stable, determine the next status
            if (is_climbing(v_forward))
            {
                if (control_mode == FLOOR)
                    set_status(BACKING);
                else
                    set_status(CLIMBING);
            } else
            {
                set_status(FORWARDING);
            }
        }
            break;
        case BACKING:
            if (_now - _status_ts > 120000)
            {
                set_error("BACKING TIMEOUT");
                break;
            }

            motors.set_forward(-0.5f);
            if (is_on_ground(v_downward))
            {
                //wait for stable
                if (_now - _step_ts < 1500)
                    break;

                set_status(TURNING);
            }
            else
            {
                _step_ts = _now;
            }
            break;
        case TURNING:
        {
            if (_now - _status_ts > 60000)
            {
                set_error("TURNING TIMEOUT");
                break;
            }

            if (fix_direction(target_direction, v_forward, v_downward))
            {
                if (_step == 0)
                {
                    _step = 1;
                    _step_ts = _now;
                }
                else if (_now - _step_ts >= 5000) //turning is stable for 5000ms
                {
                    set_status(FORWARDING);
                }
            } else
                _step = 0;
            break;
        }
        case FORWARDING:
            if (_now - _status_ts > 40000)
            {
                set_status(RAISING);
                gcs().send_text(MAV_SEVERITY_NOTICE, "FORWARDING TIMEOUT");
                return;
            }
            motors.set_forward(0.7f);

            if (is_climbing(v_forward))
            {
                if (should_wash_wall())
                {
                    set_status(RAISING);
                }
                else
                {
                    //KEEP WASHING BOTTOM
                    //Rotate right some deg to avoid wall
                    wash_rotate(target_direction, target_direction);
                    set_status(BACKING);
                }
            }
            else
            {
                fix_direction(target_direction, v_forward, v_downward);

                if (_step == 0)
                {
                    int8_t v = detect_vibration();
                    if (v > 1)
                        vib_n = 5;
                    else if (v > 0)
                        vib_n++;
                    if (vib_n >= 5)
                    {
                        if (!should_wash_wall())
                        {
                            //KEEP WASHING BOTTOM
                            //Rotate right some deg to avoid wall
                            wash_rotate(target_direction, target_direction);
                            set_status(BACKING);
                            break;
                        }
                        _step = 1;
                        _step_ts = _now;
                    }
                }
                else
                {
                    if (_now - _step_ts> 4000)
                    {
                        set_status(RAISING);
                    }
                }
            }
            break;
        case RAISING:
            motors.set_forward(1);
            if (is_on_wall(v_downward))
            {
                set_status(PENDING);
            }
            else if ((_now - _status_ts) > 10000)
            {
                //todo bypass the obstacle
                set_status(FORWARDING);
            }
            break;
        case CLIMBING:
        {
            motors.set_forward(1.0f);
            if (control_mode == PULLUP)
                return;
            uint32_t dt = _now - _status_ts;
            if (dt < 3000) //keep 3sec for stablizing
                return;

            if (is_on_wall(v_downward))
            {
                if (!should_wash_wall())
                {
                    //KEEP WASHING BOTTOM
                    //Rotate right some deg to avoid wall
                    wash_rotate(target_direction, target_direction);
                    set_status(BACKING);
                    return;
                }
                if (m.c.y <= -0.3f)
                    motors.set_yaw(0.5f);
                else if (m.c.y >= 0.3f)
                    motors.set_yaw(-0.5f);
                else
                    motors.set_yaw(0);
                if ((_now - _status_ts) > 60000)
                    set_error("No waterline detect");
                if (_step == 2)
                {
                    if (_now - _step_ts >= 2000)
                        set_status(LATERAL);
                }
                else
                {
                    if ((water_detector->read() & 1) != 0)
                    {
                        _step = 2;
                        _step_ts = _now;
                    }
                    else if (_step == 1)
                    {
                        // Detect waterline by rpm
                        if (rpm[2] > 1400 && rpm[4] > 1400)
                        {
                            _step = 2;
                            _step_ts = _now;
                        }
                    }
                    else
                    {
                        if (_now - _step_ts > 30000)
                        {
                            _step = 1;
                            _step_ts = _now;
                        }
                    }
                }
            } else
            {
                set_status(FORWARDING);
            }
        }
            break;
        case LATERAL:
        {
            motors.set_forward(0.7f);
            if (_step == 0)
            {
                //Speed up for 3sec
                if (_now - _step_ts > 3000)
                {
                    _step = 1;
                    _step_ts = _now;
                    if (_pump == LEFT_STRONG)
                        set_pump(LEFT);
                    else
                        set_pump(RIGHT);
                }
            }
            else if (_step == 1)
            {
                // After lateral moving, Pump reverse to speed down
                if (_now - _step_ts > lateral_ms)
                {
                    _step = 2;
                    _step_ts = _now;
                    if (_pump == LEFT)
                        set_pump(RIGHT);
                    else
                        set_pump(LEFT);
                }
            }
            else
            {
                // Speed down for 2sec and determine the next action
                if (_now - _step_ts > 2000)
                {
                    if (lateral_control.num < 2)
                    {
                        if (control_mode == WATERLINE)
                            set_status(LATERAL);
                        else
                        {
                            target_direction = v_downward;
                            set_status(BACKING);
                        }
                    }
                    else
                    {
                        uint32_t dt = _now - lateral_control.time_0;
                        if (control_mode == WATERLINE || dt > 240000)
                        {
                            Vector3f fwd(v_downward.x, v_downward.y, 0);
                            fwd.normalize();
                            Vector3f z(0, 0, 1);
                            if (_phase == WALL_LEFT)
                            {
                                //Turn to left by 90 deg
                                target_direction = fwd % z;
                            }
                            else
                            {
                                //Turn to right by 90 deg
                                target_direction = z % fwd;
                            }
                            lateral_control.num = 0;
                        }
                        else
                        {
                            target_direction = v_downward;
                        }
                        set_status(BACKING);
                    } // end of rotate by 90 deg
                } //end of step 3
            } //end of step 3
        }
            break;
        case ERROR:
        default:
            break;
    }
}

void Sub::set_status(STATUS status)
{
    _status = status;
    _status_ts = _now;
    _step = 0;
    _step_ts = _now;
    switch (status)
    {
        case PENDING:
            hal.shell->printf("PENDING\r\n");
            gcs().send_text(MAV_SEVERITY_NOTICE, "PENDING");
            target_direction = v_forward;
            set_pump(NORMAL);
            motors.set_yaw(0);
            motors.set_forward(0);
            break;
        case BACKING:
            set_pump(TINY);
            motors.set_yaw(0);
            hal.shell->printf("BACKING\r\n");
            gcs().send_text(MAV_SEVERITY_NOTICE, "BACKING");
            break;
        case FORWARDING:
            hal.shell->printf("FORWARDING\r\n");
            gcs().send_text(MAV_SEVERITY_NOTICE, "FORWARDING");
            set_pump(NORMAL);
            motors.set_yaw(0);
            max_dx_acc = 0;
            max_dy_acc = 0;
            pulse_n = 0;
            vib_n = 0;
            pre_acc = ins.get_accel();
            break;
        case RAISING:
            hal.shell->printf("RAISING\r\n");
            gcs().send_text(MAV_SEVERITY_NOTICE, "RAISING");
            set_pump(IDLE);
            motors.set_yaw(0);
            break;
        case CLIMBING:
            hal.shell->printf("CLIMBING\r\n");
            gcs().send_text(MAV_SEVERITY_NOTICE, "CLIMBING");
            set_pump(STRONG);
            motors.set_yaw(0);
            break;
        case LATERAL:
            if (lateral_control.num == 0)
            {
                lateral_control.num = 1;
                if (_phase == WALL_LEFT)
                    set_pump(RIGHT_STRONG);
                else
                    set_pump(LEFT_STRONG);
                lateral_control.wall_0 = v_downward;
                lateral_control.time_0 = _now;
                lateral_ms = 10000;
            }
            else
            {
                lateral_control.num++;
                if (_phase == WALL_LEFT)
                    set_pump(LEFT_STRONG);
                else
                    set_pump(RIGHT_STRONG);
                lateral_control.wall_last = v_downward;
                if (control_mode == WATERLINE)
                    lateral_ms = 30000;
                else
                    lateral_ms = 6000;
            }
            if (_pump == LEFT_STRONG)
            {
                hal.shell->printf("LEFT\r\n");
                gcs().send_text(MAV_SEVERITY_NOTICE, "LEFT");
            }
            else
            {
                hal.shell->printf("RIGHT\r\n");
                gcs().send_text(MAV_SEVERITY_NOTICE, "RIGHT");
            }

            motors.set_yaw(0);
            break;
        case TURNING:
            hal.shell->printf("TURNING\r\n");
            gcs().send_text(MAV_SEVERITY_NOTICE, "TURNING");
            set_pump(NORMAL);
            motors.set_forward(0);
            if (!target_direction.is_zero())
            {
                target_direction.z = 0;
                target_direction.normalize();
            }
            break;
        case ERROR:
            set_pump(IDLE);
            motors.set_forward(0);
            motors.set_yaw(0);
            break;
        case PAUSE:
            motors.set_forward(0);
            motors.set_yaw(0);
            _is_manual_pump = false;
            set_pump(NORMAL);
            break;
    }
}

void Sub::set_error(const char *msg)
{
    set_status(ERROR);
    hal.shell->printf("ERROR: %s\r\n", msg);
    gcs().send_text(MAV_SEVERITY_NOTICE, msg);
    return;
}

// All parameters must be normalized
bool Sub::fix_direction(const Vector3f &target, const Vector3f &forward, const Vector3f &downward)
{
    Vector3f l = target%downward;
    float len = l.length();
    if (len < SIN_45)
        //target is too near to downward, no need to fix direction
        return true;
    l /= len; //normalize
    float sin = l*forward;
    float cos0 = target*forward;
    if (cos0 > 0)
    {
        if (sin > 0)
        {
            if (sin < SIN_10)
                return true;
            else if (sin < SIN_45)
                motors.set_yaw(0.3);
            else
                motors.set_yaw(0.5);
        }
        else
        {
            if (sin > -SIN_10)
                return true;
            else if (sin > -SIN_45)
                motors.set_yaw(-0.3);
            else
                motors.set_yaw(-0.5);
        }
    }
    else
    {
        // Opposite direction, turn a big angle
        if (sin < 0)
            motors.set_yaw(-1);
        else
            motors.set_yaw(1);
    }

    return false;
}


bool Sub::should_wash_wall(void)
{
    switch (control_mode)
    {
        case FLOOR:
            return false;
        case ULTRA:
        case REGULAR:
        case FAST:
            return _phase == WALL_LEFT || _phase == WALL_RIGHT;
        default:
            return true;
    }
}

void Sub::wash_rotate(const Vector3f &fwd, Vector3f &target)
{
    //cos(a + b) = cos(a)cos(b) - sin(a)sin(b)
    //sin(a + b) = sin(a)cos(b) + cos(a)sib(b)
    float cos = cosf(radians(_rotate_degree));
    float sin = sqrtf(1 - cos*cos);
    float x = fwd.x*cos - fwd.y*sin;
    float y = fwd.y*cos + fwd.x*sin;
    target.x = x;
    target.y = y;
    target.z = 0;
    target.normalize();

    _rotate_degree += 7;
    if (_rotate_degree > 160)
    {
        _rotate_degree = (_rotate_degree-110)%50 + 110;
    }
};

int8_t Sub::detect_vibration(void)
{
    const Vector3f &acc = ins.get_accel();
    float dx = fabsf(acc.x - pre_acc.x);
    float dy = fabsf(acc.y - pre_acc.y);
    pre_acc = acc*0.239 + pre_acc*0.761;
    if (dx > pulse_thr || dy > pulse_thr)
    {
        pulse_n++;
        if (max_dx_acc < dx)
            max_dx_acc = dx;
        if (max_dy_acc < dy)
            max_dy_acc = dy;
    }
    else if (pulse_n)
    {
        if (pulse_n > pulse_thn)
        {
            pulse_n = 0;
            if (max_dx_acc < 0.4 && max_dy_acc < 0.4)
                return 1;
            else
                return 2;
        }
    }

    return 0;
}

void Sub::set_pump(PUMP_STATE state)
{
    _pump = state;
    pump3 = 1500;
    pump4 = 1500;
    switch (state)
    {
        case IDLE:
            pump1 = 1500;
            pump2 = 1500;
            break;
        case NORMAL:
            pump1 = 1750;
            pump2 = 1750;
            break;
        case STRONG:
            pump1 = 1800;
            pump2 = 1800;
            break;
        case LEFT_STRONG:
            pump1 = 1500;
            pump2 = 1950;
            break;
        case LEFT:
            pump1 = 1560;
            pump2 = 1850;
            break;
        case RIGHT_STRONG:
            pump1 = 1950;
            pump2 = 1500;
            break;
        case RIGHT:
            pump1 = 1850;
            pump2 = 1560;
            break;
        case TINY:
            pump1 = 1560;
            pump2 = 1560;
            break;
    }
}

static void set_ch_pwm(SRV_Channel *chan, int16_t pwm)
{
    uint16_t p = chan->get_output_pwm();
    int16_t t = chan->get_reversed() ? 3000 - pwm : pwm;
    int16_t d = t - p;
    const int16_t DELTA = 20;
    if (d > 0)
    {
        if (d > DELTA)
            chan->set_output_pwm(p + DELTA);
        else
            chan->set_output_pwm(t);
    }
    else if (d < 0)
    {
        if (d < -DELTA)
            chan->set_output_pwm(p - DELTA);
        else
            chan->set_output_pwm(t);
    }
}

void Sub::smoothly_control_pump(void)
{
    set_ch_pwm(ch1, pump1);
    set_ch_pwm(ch2, pump2);
    set_ch_pwm(ch3, pump3);
    set_ch_pwm(ch4, pump4);
}

bool Sub::handle_do_pause_continue(mavlink_command_long_t command) {
    if (is_zero(command.param1))
    {
        set_status(PAUSE);
    }
    else if(_status == PAUSE)
    {
        set_status(PENDING);
    }

    return true;
}

