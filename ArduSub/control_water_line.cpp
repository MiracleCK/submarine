#include "Sub.h"

#define WL_DEBUG(m) hal.shell->printf("%s\r\n")
#define WATER_LINE_WASH_TIME_LENGTH 10000

typedef struct {
    Vector3f orientation1;
    Vector3f orientation2;
    uint32_t wash_left_time_ms;
    uint32_t wash_right_time_ms;
} Task;

enum STATE {
    STATE_BACK_TO_BOTTOM,
    STATE_DETECTING,
    STATE_WASH_LEFT,
    STATE_WASH_RIGHT
};

STATE state;
Task tasks[4];
uint32_t task_index;
bool wl_fatal_error;
uint32_t rotation_ts = 0;
uint32_t detection_ts = 0;
uint32_t backing_ts = 0;
float lpf = 0;
uint32_t now;
Vector3f *target_orientation;
const Quaternion UP_STRAIGHT(0.7071, 0, 0.7071, 0);
Quaternion quat;
Vector3f forward, downward, temp;

bool Sub::water_line_init()
{
    for (int i = 0; i < 4; ++i) {
        tasks[i].orientation1.zero();
        tasks[i].orientation2.zero();
        tasks[i].wash_left_time_ms = 0;
        tasks[i].wash_right_time_ms = 0;
    }
    task_index = 0;
    wl_fatal_error = false;
    target_orientation = NULL;
    state = STATE_BACK_TO_BOTTOM;
    rotation_ts = 0;
    detection_ts = 0;
    backing_ts = 0;
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Sub::water_line_run()
{
    if (!motors.armed() || wl_fatal_error) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        state = STATE_BACK_TO_BOTTOM;
        target_orientation = NULL;
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    const Matrix3f &m = ahrs.get_rotation_body_to_ned();
    forward.x = m.a.x;
    forward.y = m.b.x;
    forward.z = m.c.x;
    downward.x = m.a.z;
    downward.y = m.b.z;
    downward.z = m.c.z;
    now = AP_HAL::millis();

    switch (state) {
        case STATE_DETECTING:
            detect();
            break;
        case STATE_WASH_LEFT:
            wash_left();
            break;
        case STATE_WASH_RIGHT:
            wash_right();
            break;
        case STATE_BACK_TO_BOTTOM:
        default:
            back_to_bottom();
            break;
    }
}

void set_state(STATE st, const char *msg)
{
    state = st;
    WL_DEBUG(msg);
}

void vector_to_quat(Vector3f &v, Quaternion &q)
{
    Matrix3f m;
    m.a.x = v.x;
    m.a.y = v.y;
    m.a.z = 0;
    m.a.normalize();

    m.c.x = 0;
    m.c.y = 0;
    m.c.z = 1;

    m.b = (m.c)%(m.a);

    q.from_rotation_matrix(m);
}

bool Sub::turning_orientation(Vector3f &target)
{
    //<15 degree
    return target*forward/target.length() > 0.97f;
}

void Sub::detect(void)
{
    if(detection_ts == 0)
    {
        detection_ts = now;
    }
    else if (now - detection_ts > 180000)
    {
        set_state(STATE_BACK_TO_BOTTOM, "DETECT TIMEOUT");
    }

    if(downward.z < 0.25f && downward.z > -0.25f) //climbing wall
    {
        motors.set_forward(1);
        motors.set_lateral(0);
        if(forward.z < -0.9)
        {
            //not straight climbing up (15+ degrees), fix attitude
            //TODO: CHECK UP_STRAIGHT is ok for quaternion
            attitude_control.input_quaternion(UP_STRAIGHT);
        }

        if(water_detector.read()&1)
        {
            //Arrive at water line
            tasks[task_index].wash_left_time_ms = now;
            tasks[task_index].orientation1(downward.x, downward.y, 0);
            set_state(STATE_WASH_LEFT, "SWITCH TO LEFT");
            detection_ts = 0;
        }
    }
    else if(target_orientation != NULL)
    {
        //change orientation
        if(rotation_ts == 0)
        {
            vector_to_quat(*target_orientation, quat);
            attitude_control.input_quaternion(UP_STRAIGHT);
            rotation_ts = AP_HAL::millis();
        }
        //it is rotating, keep going
        if(turning_orientation(*target_orientation))
        {
            target_orientation = NULL;
            attitude_control.relax_attitude_controllers();
            rotation_ts = 0;
        }
        else
        {
            if(now - rotation_ts > 10000)
            {
                wl_fatal_error = true;
                set_state(STATE_BACK_TO_BOTTOM, "TURNING TIMEOUT");
                detection_ts = 0;
            }
        }
    }
    else
    {
        //going forward
        motors.set_forward(0.8f);
        motors.set_lateral(0);
        attitude_control.relax_attitude_controllers();
    }
}

void Sub::wash_left(void)
{
    Task &t = tasks[task_index];
    if(now - t.wash_left_time_ms > WATER_LINE_WASH_TIME_LENGTH)
    {
        tasks[task_index].wash_right_time_ms = now;
        set_state(STATE_WASH_RIGHT, "SWITCH TO RIGHT");
        attitude_control.relax_attitude_controllers();
        motors.set_lateral(0);
        return;
    }

    t.orientation2.x = downward.x;
    t.orientation2.y = downward.y;
    t.orientation2.z = 0;

    if (downward.z < -0.9f || downward.z > 0.9f)
    {
        //Robot is dropped from wall
        //incline < 25 deg to straight downward
        target_orientation = &t.orientation2;
        set_state(STATE_DETECTING, "DROPPED");
        return;
    }
    else if(water_detector.read()&1)
    {
        //sinking, to climbing stronger
        motors.set_forward(1.0f);
    }
    else
    {
        motors.set_forward(0.8f);
    }
    attitude_control.relax_attitude_controllers();
    motors.set_lateral(-0.5f);
}

void Sub::wash_right(void)
{
    Task &t = tasks[task_index];
    if(now - t.wash_right_time_ms > WATER_LINE_WASH_TIME_LENGTH)
    {
        Vector3f z(0, 0, 1);
        temp = z%t.orientation2;
        target_orientation = &temp;
        set_state(STATE_BACK_TO_BOTTOM, "BACK TO BOTTOM");
        attitude_control.relax_attitude_controllers();
        motors.set_lateral(0);
        return;
    }

    t.orientation2.x = downward.x;
    t.orientation2.y = downward.y;
    t.orientation2.z = 0;

    if (downward.z < -0.9f || downward.z > 0.9f)
    {
        //incline < 25 deg to straight downward
        //Robot is dropped from wall
        target_orientation = &t.orientation2;
        set_state(STATE_DETECTING, "DROPPED");
        return;
    }
    else if(water_detector.read()&1)
    {
        //sinking, to climbing stronger
        motors.set_forward(1.0f);
    }
    else
    {
        motors.set_forward(0.8f);
    }
    attitude_control.relax_attitude_controllers();
    motors.set_lateral(0.5f);
}

void Sub::back_to_bottom(void)
{
    if (backing_ts == 0)
    {
        backing_ts = now;
        lpf = forward.z;
    }
    else if (now - backing_ts > 30000)
    {
        wl_fatal_error = true;
        set_state(STATE_BACK_TO_BOTTOM, "BACKING TIMEOUT");
        backing_ts = 0;
    }

    lpf = 0.01*forward.z + 0.99*lpf;

    if (downward.z < 0.25 && downward.z > -0.25)
    {
        //on the wall
        if(forward.z < -0.9)
        {
            //not straight erect (15+ degrees), fix attitude
            //TODO: CHECK UP_STRAIGHT is ok for quaternion
            attitude_control.input_quaternion(UP_STRAIGHT);
            motors.set_forward(0);
            motors.set_lateral(0);
        }
        else
        {
            //move back to bottom
            motors.set_forward(-0.8);
            motors.set_lateral(0);
        }
    }
    else
    {
        if (lpf - forward.z < 0.1)
        {
            //to the bottom
            if (target_orientation != NULL)
            {
                Task &t = tasks[task_index];
                Vector3f v1 = t.orientation2;
                Vector3f v2 = *target_orientation;
                if (v1*v2/v1.length()/v2.length() < 0.1)
                {
                    if(++task_index >= 4)
                    {
                        wl_fatal_error = true;
                        set_state(STATE_BACK_TO_BOTTOM, "All is done");
                    }
                    else
                    {
                        set_state(STATE_DETECTING, "TO THE NEXT");
                    }
                }
                else
                {
                    set_state(STATE_DETECTING, "WASH AGAIN");
                }
            }
            else
            {
                wl_fatal_error = true;
                set_state(STATE_BACK_TO_BOTTOM, "NO TARGET");
            }
            backing_ts = 0;
        }
        else
        {
            motors.set_forward(-0.8);
            motors.set_lateral(0);
        }
    }
}
