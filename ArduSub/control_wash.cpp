#include "Sub.h"

#define SQRT3_2 0.866f
bool Sub::wash_init(control_mode_t mode)
{
    v_desire_direction.zero();
    _now = AP_HAL::millis();
    mode_sum_ms = 0;
    mode_active_ts = 0;
    step_ms = 0;
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
    set_status(UNKNOWN);
    _step = WALL_RIGHT;
    return true;
}

void Sub::wash_run(void)
{
    _now = AP_HAL::millis();
    if (!motors.armed())
    {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0, true, g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        SRV_Channels::set_output_pwm(SRV_Channel::k_throttleLeft, 1500);
        SRV_Channels::set_output_pwm(SRV_Channel::k_throttleRight, 1500);
        SRV_Channels::set_output_pwm(SRV_Channel::k_elevon_left, 1500);
        SRV_Channels::set_output_pwm(SRV_Channel::k_elevon_right, 1500);
        mode_active_ts = 0;
        if (!is_zero(channel_forward->norm_input())) {
            arming.arm(AP_Arming::Method::AUXSWITCH, false);
        }
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (_status == PAUSE)
    {
        mode_active_ts = 0;
        motors.set_yaw(channel_yaw->norm_input());
        motors.set_forward(channel_forward->norm_input());
        SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleLeft, channel_left_pump->get_radio_in());
        SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleRight, channel_right_pump->get_radio_in());
        if (channel_pump->get_radio_in() != 1500)
        {
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_left, channel_pump->get_radio_in());
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_right, channel_pump->get_radio_in());
        }
        else
        {
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_left, channel_pump1->get_radio_in());
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_right, channel_pump2->get_radio_in());
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
        step_ms += dt;
    }

    if (mode_sum_ms >= mode_max_ms)
    {
        motors.armed(false);
        mode_sum_ms = mode_max_ms;
        gcs().send_text(MAV_SEVERITY_INFO, "Task is done");
        if (control_mode == PULLUP)
        {
            set_mode(prev_control_mode, ModeReason::TERMINATE);
        }
        return;
    }
    mode_active_ts = _now;

    const Matrix3f &m = ahrs.get_rotation_body_to_ned();
    v_forward.x = m.a.x;
    v_forward.y = m.b.x;
    v_forward.z = m.c.x;
    v_downward.x = m.a.z;
    v_downward.y = m.b.z;
    v_downward.z = m.c.z;

    if (control_mode != FLOOR)
    {
        if (_step == WALL_RIGHT && mode_sum_ms > mode_max_ms/2)
            _step = BOTTOM;
    }

    switch (_status)
    {
        case UNKNOWN:
            if (_now - _status_ms < 7000)
                break;
            if (is_on_wall(v_downward))
            {
                if (control_mode == FLOOR)
                    set_status(BACKING);
                else
                    set_status(CLIMBING);
            }
            else
            {
                set_status(FORWARDING);
            }
            break;
        case RELAXING:
            if (_now - _status_ms > 30000)
            {
                set_error("RELAXING TIMEOUT");
                return;
            }

            if (is_standing_straight(v_downward))
                set_status(FORWARDING);
            break;
        case BACKING:
            if (_now - _status_ms > 60000)
            {
                set_error("BACKING TIMEOUT");
                return;
            }

            if (v_desire_direction.is_zero())
            {
                if (!is_on_wall(v_downward))
                {
                    set_status(FORWARDING);
                    return;
                }
            }
            else if (is_standing_straight(v_downward))
            {
                if (_now - _status_ms > 10)
                {
                    if (_delay_ms == 0){
                        _delay_ms = _now;
                        return;
                    }

                    if (_now - _delay_ms < 1000)
                        return;
                }

                set_status(TURNING);
                return;
            }
            else
            {
                _delay_ms = 0;
            }
            motors.set_forward(-0.5f);
            break;
        case TURNING:
        {
            if (_now - _status_ms > 60000)
            {
                set_error("TURNING TIMEOUT");
                return;
            }

            if (v_desire_direction.is_zero())
            {
                set_status(FORWARDING);
                return;
            }

            if (turning_orientation(v_desire_direction, v_forward))
            {
                if (_delay_ms == 0)
                    _delay_ms = _now;
                else if (_now - _delay_ms >= 500) //turning is stable for 500ms
                {
                    v_desire_direction.zero();
                    step_ms = 0;
                    set_status(FORWARDING);
                }
            } else
                _delay_ms = 0;
            break;
        }
        case FORWARDING:
            if (_now - _status_ms > 300000)
            {
                set_error("FORWARDING TIMEOUT");
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
                    wash_rotate(v_forward_target, v_desire_direction);
                    hal.shell->printf("desire:(%.2f %.2f)\r\n",
                                      v_desire_direction.x,
                                      v_desire_direction.y);
                    set_status(BACKING);
                }
            }
            else
            {
                const Vector3f &acc = AP::ins().get_accel();
                if (is_standing_straight(v_downward))
                {
                    turning_orientation(v_forward_target, v_forward);
                    float d1 = acc.x - pre_acc.x;
                    float d2 = acc.y - pre_acc.y;
                    float sa = d1*d1 + d2*d2;
                    if (sa > pulse_thr) {
                        pulse_n++;
                        hal.shell->printf("%0.2f\r\n", sa);
                        pre_acc = acc;
                        return;
                    }
                }
                else
                {
                    //horizontal moving on wall
                    if (is_on_wall(v_downward)) {
                        if (m.c.y > 0)
                            motors.set_yaw(0.3f);
                        else
                            motors.set_yaw(-0.3f);
                    }
                    else {
                        motors.set_yaw(0);
                    }
                }

                if (pulse_n)
                {
                    if(pulse_n <= 300 && pulse_n >= pulse_thn) {
                        gcs().send_text(MAV_SEVERITY_INFO, "Collision");
                        pulse_n = 0;
                        if (!should_wash_wall())
                        {
                            //KEEP WASHING BOTTOM
                            //Rotate right some deg to avoid wall
                            wash_rotate(v_forward_target, v_desire_direction);
                            hal.shell->printf("desire:(%.2f %.2f)\r\n",
                                              v_desire_direction.x,
                                              v_desire_direction.y);
                            set_status(BACKING);
                            return;
                        }
                        _delay_ms = _now;
                    }
                    else
                        pulse_n--;
                }
                pre_acc = acc;
                if (_delay_ms && (_now - _delay_ms) > 5000)
                    set_status(RAISING);
            }
            break;
        case RAISING:
            motors.set_forward(1);
            if (is_on_wall(v_downward))
            {
                set_status(CLIMBING);
            }
            else if ((_now - _status_ms) > 10000)
            {
                set_status(FORWARDING);
            }
            break;
        case CLIMBING:
            motors.set_forward(1.0f);
            if (control_mode == PULLUP)
                return;
            if ((_now - _status_ms) > 180000)
            {
                set_error("Timeout");
            }

            if (is_on_wall(v_downward))
            {
                if (m.c.y <= -0.7f)
                    motors.set_yaw(0.5f);
                else if (m.c.y >= 0.7f)
                    motors.set_yaw(-0.5f);
                else
                    motors.set_yaw(0);
                if ((_now - _status_ms) > 60000)
                    set_error("No waterline detect");
                if ((_now - _status_ms) > 15000)
                    set_status(WASH_LATERAL);
                if((water_detector->read()&1) != 0)
                {
                    if (_delay_ms == 0)
                        _delay_ms = _now;
                    else if (_now - _delay_ms >= 2000)
                        set_status(WASH_LATERAL);
                }
                else
                    _delay_ms = 0;
            }
            else
            {
                set_status(FORWARDING);
            }
            break;
        case WASH_LATERAL:
        {
            if (control_mode == WATERLINE)
            {
                uint32_t dt = _now - _status_ms;
                if (dt < 30000)
                    motors.set_forward(0.7f);
                else if (dt < 32000)
                {
                    if (_pump != NORMAL)
                        set_pump(NORMAL);
                    motors.set_forward(0.5f);
                }
                else
                {
                    Vector3f fwd(v_downward.x, v_downward.y, 0);
                    fwd.normalize();
                    Vector3f z(0, 0, 1);
                    if (_step == WALL_LEFT)
                    {
                        //Turn to left by 90 deg
                        v_desire_direction = fwd % z;
                    }
                    else
                    {
                        //Turn to right by 90 deg
                        v_desire_direction = z % fwd;
                    }
                    set_status(BACKING);
                }
            }
            else
            {
                uint32_t dt = _now - _status_ms;
                if (dt < 10000)
                    motors.set_forward(0.7f);
                else if (dt < 12000)
                {
                    if (_pump != NORMAL)
                        set_pump(NORMAL);
                    motors.set_forward(0.7f);
                }
                else
                {
                    if (step_ms > 120000)
                    {
                        wash_rotate(v_forward_target, v_desire_direction);
                    }
                    else
                        v_desire_direction.zero();
                    set_status(BACKING);
                }
            }
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
    _status_ms = _now;
    _delay_ms = 0;
    switch (status)
    {
        case UNKNOWN:
            hal.shell->printf("UNKNOWN\r\n");
            gcs().send_text(MAV_SEVERITY_INFO, "UNKNOWN");
            set_pump(NORMAL);
            break;
        case BACKING:
            set_pump(TINY);
            motors.set_yaw(0);
            hal.shell->printf("BACKING\r\n");
            gcs().send_text(MAV_SEVERITY_INFO, "BACKING");
            break;
        case FORWARDING:
            hal.shell->printf("FORWARDING\r\n");
            gcs().send_text(MAV_SEVERITY_INFO, "FORWARDING");
            v_forward_target = v_forward;
            v_forward_target.z = 0;
            v_forward_target.normalize();
            set_pump(NORMAL);
            motors.set_yaw(0);
            break;
        case RAISING:
            hal.shell->printf("RAISING\r\n");
            gcs().send_text(MAV_SEVERITY_INFO, "RAISING");
            set_pump(IDLE);
            motors.set_yaw(0);
            break;
        case CLIMBING:
            hal.shell->printf("CLIMBING\r\n");
            gcs().send_text(MAV_SEVERITY_INFO, "CLIMBING");
            set_pump(STRONG);
            motors.set_yaw(0);
            break;
        case WASH_LATERAL:
            if (_step == WALL_LEFT)
            {
                set_pump(LEFT);
                hal.shell->printf("LEFT\r\n");
                gcs().send_text(MAV_SEVERITY_INFO, "LEFT");
            }
            else
            {
                set_pump(RIGHT);
                hal.shell->printf("RIGHT\r\n");
                gcs().send_text(MAV_SEVERITY_INFO, "RIGHT");
            }
            motors.set_yaw(0);
            break;
        case TURNING:
            hal.shell->printf("TURNING\r\n");
            gcs().send_text(MAV_SEVERITY_INFO, "TURNING");
            set_pump(NORMAL);
            motors.set_forward(0);
            if (!v_desire_direction.is_zero())
            {
                v_desire_direction.z = 0;
                v_desire_direction.normalize();
            }
            break;
        case RELAXING:
            set_pump(NORMAL);
            motors.set_forward(0);
            motors.set_yaw(0);
            break;
        case ERROR:
            set_pump(IDLE);
            motors.set_forward(0);
            motors.set_yaw(0);
            break;
        case PAUSE:
            motors.set_forward(0);
            motors.set_yaw(0);
            break;
    }
}

void Sub::set_error(const char *msg)
{
    set_status(ERROR);
    hal.shell->printf("ERROR: %s\r\n", msg);
    gcs().send_text(MAV_SEVERITY_INFO, msg);
    return;
}

bool Sub::turning_orientation(const Vector3f &target, const Vector3f &forward)
{
    Vector3f yaw(forward.x, forward.y, 0);
    yaw.normalize();
//    float target_yaw_rate;
    float cos = target * yaw;
    if (cos > 0.99f) { // < 8.1 deg
        //Direction achieved
        motors.set_yaw(0);
        return true;
    } else {
        Vector3f k = yaw % target;
        float sin = k.z;
        if (cos < 0.966f) { // Opposite direction, turn big angle
            if (sin < 0.0f)
            {
//                target_yaw_rate = get_pilot_desired_yaw_rate(-1);
//                attitude_control.rate_bf_yaw_target(-1);
                motors.set_yaw(-1);
            }
            else {
//                target_yaw_rate = get_pilot_desired_yaw_rate(1);
//                attitude_control.rate_bf_yaw_target(1);
                motors.set_yaw(1);
            }
        } else { //turn small angle
//            target_yaw_rate = get_pilot_desired_yaw_rate(k.z);
//            attitude_control.rate_bf_yaw_target(k.z);
            motors.set_yaw(sin > 0 ? 0.5f : -0.5f);
            if (sin < 0.25f && sin > -0.25f) // <15deg
                return true;
        }
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
            return _step == WALL_LEFT || _step == WALL_RIGHT;
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
    target.x = fwd.x*cos - fwd.y*sin;
    target.y = fwd.y*cos + fwd.x*sin;
    target.z = 0;

    _rotate_degree += 7;
    if (_rotate_degree > 160)
    {
        _rotate_degree = (_rotate_degree-110)%50 + 110;
    }
};

void Sub::set_pump(PUMP_STATE state)
{
    _pump = state;
    switch (state)
    {
        case IDLE:
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleLeft, 1500);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleRight, 1500);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_left, 1500);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_right, 1500);
            break;
        case NORMAL:
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleLeft, 1580);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleRight, 1580);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_left, 1550);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_right, 1550);
            break;
        case STRONG:
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleLeft, 1600);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleRight, 1600);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_left, 1800);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_right, 1800);
            break;
        case LEFT:
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleLeft, 1500);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleRight, 1800);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_left, 1600);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_right, 1600);
            break;
        case RIGHT:
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleLeft, 1800);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleRight, 1500);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_left, 1600);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_right, 1600);
            break;
        case TINY:
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleLeft, 1500);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_throttleRight, 1500);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_left, 1540);
            SRV_Channels::set_output_pwm_trimmed(SRV_Channel::k_elevon_right, 1540);
            break;
    }
}

bool Sub::handle_do_pause_continue(mavlink_command_long_t command) {
    if (is_zero(command.param1))
    {
        set_status(PAUSE);
    }
    else if(_status == PAUSE)
    {
        set_status(UNKNOWN);
    }

    return true;
}

