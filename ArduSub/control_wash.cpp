#include "Sub.h"

#define SQRT3_2 0.866f

bool Sub::wash_init(void)
{
    v_desire_direction.zero();
    _mode_ms = AP_HAL::millis();
    if (control_mode == PULLUP)
        set_status(FORWARDING);
    else
        set_status(BACKING);
    _step = WALL;
    return true;
}

void Sub::wash_run(void)
{
    if (!motors.armed())
    {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0, true, g.throttle_filt);
        attitude_control.relax_attitude_controllers();

        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const Matrix3f &m = ahrs.get_rotation_body_to_ned();
    v_forward.x = m.a.x;
    v_forward.y = m.b.x;
    v_forward.z = m.c.x;
    v_downward.x = m.a.z;
    v_downward.y = m.b.z;
    v_downward.z = m.c.z;
    uint32_t now = AP_HAL::millis();

    if (control_mode != FLOOR)
    {
        if (_step == WALL && now - _status_ms > 60000 && _status == FORWARDING)
            _step = BOTTOM;
    }

    if (now - _status_ms > 120000)
    {
        set_error("Washing finished");
        return;
    }

    switch (_status)
    {
        case BACKING:
            if (now - _status_ms > 30000)
            {
                set_error("BACKING TIMEOUT");
                return;
            }

            if (is_standing_straight(v_downward))
            {
                motors.set_forward(0);
                if (v_desire_direction.is_zero())
                    set_status(FORWARDING);
                else
                    set_status(TURNING);
            } else
            {
                attitude_control.input_quaternion(UP_STRAIGHT);
                motors.set_forward(-0.5f);
            }
            break;
        case TURNING:
        {
            if (now - _status_ms > 20000)
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
                if (_action_ms == 0)
                    _action_ms = now;
                else if (now - _action_ms >= 500) //turning is stable for 500ms
                {
                    v_desire_direction.zero();
                    set_status(FORWARDING);
                }
            } else
                _action_ms = 0;
            break;
        }
        case FORWARDING:
            if (now - _status_ms > 30000)
            {
                set_error("FORWARDING TIMEOUT");
                return;
            }
            motors.set_forward(1);
            if (control_mode == PULLUP)
            {
                //Keep forwarding and be pulled up
                _status_ms = now;
                return;
            }

            if (is_on_wall(v_downward))
            {
                if (should_wash_wall())
                {
                    motors.set_forward(0);
                    set_status(WASH_LEFT);
                }
                else
                {
                    //KEEP WASHING BOTTOM
                    //Rotate right 120 deg to avoid wall
                    //cos(a + 120) = cos(a)(-0.5) - sin(a)(sqrt3/2)
                    //sin(a + 120) = sin(a)(-0.5) + cos(a)(sqrt3/2)
                    v_desire_direction.x = -0.5*v_downward.x - SQRT3_2*v_downward.y;
                    v_desire_direction.y = -0.5*v_downward.y + SQRT3_2*v_downward.x;
                    v_desire_direction.z = 0;
                    set_status(BACKING);
                }
            }
            else if((!should_wash_wall()) && is_standing_straight(v_downward))
            {
                // Maybe a wall could change forwarding orientation, turn and
                // move away from the wall
                float gz = ahrs.get_gyro().z;
                if (gz < -0.7f || gz > 0.7f)
                {
                    _last_big_yaw_rate = gz;
                    _stable_ms = now;
                }
                else
                {
                    //When orientation is stable for 2 seconds,
                    //turn away to move off wall
                    if (_stable_ms != 0 && now - _stable_ms > 2000)
                    {
                        if (_last_big_yaw_rate < 0.7f)
                        {
                            //Turn to left by 90 deg
                            Vector3f z(0, 0, 1);
                            v_desire_direction = v_forward%z;
                            set_status(TURNING);
                        }
                        else
                        {
                            //Turn to right by 90 deg
                            Vector3f z(0, 0, 1);
                            v_desire_direction = z%v_forward;
                            set_status(TURNING);
                        }
                        _stable_ms = 0;
                    }
                }
            }
            break;
        case WASH_LEFT:
        case WASH_RIGHT:
        {
            float lateral = _status == WASH_LEFT ? -0.5f : 0.5f;
            if (now - _status_ms > 30000)
            {
                if (!is_on_wall(v_downward) || water_detector.read())
                {
                    motors.set_forward(0);
                    motors.set_lateral(0);
                    if (_status == WASH_LEFT)
                        set_status(WASH_RIGHT);
                    else
                    {
                        //Turn to right
                        Vector3f fwd(v_downward.x, v_downward.y, 0);
                        fwd.normalize();
                        Vector3f z(0, 0, 1);
                        v_desire_direction = z % fwd;
                        set_status(BACKING);
                    }

                    return;
                }
                if (_action_ms != 0 && now - _action_ms > 15000)
                {
                    motors.set_forward(0);
                    motors.set_lateral(0);
                    set_error("WALL TIMEOUT\r\n");
                    return;
                }
            }
            v_desire_direction = v_downward;
            attitude_control.input_quaternion(UP_STRAIGHT);
            if (motors.get_forward() < -0.1f)
            {
                //moving down to bottom
                if (is_on_wall(v_downward))
                {
                    _action_ms = 0;
                }
                else
                {
                    if (_action_ms == 0)
                    {
                        _action_ms = now;
                    }
                        //keep backing for 1s
                    else if (now - _action_ms >= 1000)
                    {
                        //MOVING UP
                        motors.set_forward(1);
                        _action_ms = now;
                        return;
                    }
                }
                motors.set_forward(-0.5f);
            }
            else if (motors.get_forward() > 0.1f)
            {
                motors.set_forward(1);
                if (control_mode == WATERLINE)
                {
                    //move lateral
                    if (_action_ms == 0)
                        motors.set_lateral(lateral);
                    else if (water_detector.read())
                    {
                        //arrive at surface
                        _action_ms = 0;
                    }

                }
                else
                {
                    if (is_zero(motors.get_lateral()))
                    {
                        if (water_detector.read())
                        {
                            //arrive at surface
                            _action_ms = 0;
                            _stable_ms = now;
                        }

                        if (_action_ms == 0 && now - _stable_ms > 1000)
                        {
                            //wait for 1s and move laterally
                            _stable_ms = now;
                            motors.set_lateral(lateral);
                        }
                    }
                    else if (now - _stable_ms > 2000)
                    {
                        //Prepare to move down
                        motors.set_forward(0);
                        motors.set_lateral(0);
                        _stable_ms = now;
                    }
                }
            }
            else if (now - _stable_ms > 1000)
            {
                //wait for 1s and moving down
                motors.set_forward(-0.5f);
                motors.set_lateral(0);
                _stable_ms = 0;
                _action_ms = now;
            }
        }
            break;
        case ERROR:
            break;
    }
}

void Sub::set_status(STATUS status)
{
    _status = status;
    _status_ms = AP_HAL::millis();
    _notify_ms = 0;
    _action_ms = 0;
    _stable_ms = 0;
    switch (status)
    {
        case TURNING:
            if (!v_desire_direction.is_zero())
            {
                v_desire_direction.z = 0;
                v_desire_direction.normalize();
            }
            break;
        default:
            break;
    }
}

void Sub::set_error(const char *msg)
{
    set_status(ERROR);
    hal.shell->printf("%s\r\n", msg);
    return;
}

bool Sub::turning_orientation(const Vector3f &target, const Vector3f &forward)
{
    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    Vector3f yaw(forward.x, forward.y, 0);
    yaw.normalize();
    float target_yaw_rate;
    float cos = target * yaw;
    if (cos > 0.999f) { // < 2.5 deg
        attitude_control.relax_attitude_controllers();
        //Direction achieved
        return true;
    } else {
        Vector3f k = yaw % target;
        if (cos < 0) { // Opposite direction, turn big angle
            if (k.z < 0.08f)
            {
                target_yaw_rate = get_pilot_desired_yaw_rate(-1);
                attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, target_yaw_rate);
            }
            else {
                target_yaw_rate = get_pilot_desired_yaw_rate(1);
                attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, target_yaw_rate);
            }
        } else { //turn small angle
            target_yaw_rate = get_pilot_desired_yaw_rate(k.z);
            attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, target_yaw_rate);
            motors.set_yaw(k.z);
            if (cos > 0.995f)
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
            return _step == WALL;
        default:
            return true;
    }
}

void Sub::control_lateral()
{
    if (!isnan(_target_lateral))
    {
        float l = motors.get_lateral();
        if (_target_lateral - l > 0.001f)
        {
            motors.set_lateral(l + 0.001f);
        }
        else if (_target_lateral - l < -0.001f)
        {
            motors.set_lateral(l - 0.001f);
        }
        else
        {
            motors.set_lateral(_target_lateral);
            _target_lateral = std::numeric_limits<float>::signaling_NaN();
        }
    }
}

