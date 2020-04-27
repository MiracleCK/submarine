
#include "Sub.h"

void Sub::thrust_decomposition_att_error(Matrix3f body_to_ned, Vector3f thrusts, Vector3f& thrust_decomp) {
    // NED quat is [1 0 0 0] and matrix is
    //  -     -
    // | 1 0 0 |
    // | 0 1 0 |
    // | 0 0 1 |
    //  -     -
    // Would align N axis Z to B axis Z
    // so transpose matrix and select column 3, Vector3f(0.0f, 0.0f, 1.0f)
    Vector3f att_from_vec(0.0f, 0.0f, 1.0f);
    Vector3f att_to_vec = body_to_ned.transposed() * Vector3f(0.0f, 0.0f, 1.0f);

    Vector3f vec_cross = att_from_vec % att_to_vec;
    float vec_dot = constrain_float(att_from_vec * att_to_vec, -1.0f, 1.0f);

    Quaternion vec_quat;

    float eps = 1e-5f;
    if (vec_cross.length() < eps && vec_dot < 0) {
        vec_cross.x = fabsf(att_from_vec.x);
        vec_cross.y = fabsf(att_from_vec.y);
        vec_cross.z = fabsf(att_from_vec.z);
        if (vec_cross.x < vec_cross.y) {
            if (vec_cross.x < vec_cross.z) {
                vec_cross.x = 1;
                vec_cross.y = vec_cross.z = 0;
            } else {
                vec_cross.x = vec_cross.y = 0;
                vec_cross.z = 1;
            }
        } else {
            if (vec_cross.y < vec_cross.z) {
                vec_cross.x = vec_cross.z = 0;
                vec_cross.y = 1;
            } else {
                vec_cross.x = vec_cross.y = 0;
                vec_cross.z = 1;
            }
        }
        
        vec_quat.q1 = 0;
        vec_cross = att_from_vec % vec_cross;
    } else {
        vec_quat.q1 = vec_dot + sqrtf(att_from_vec.length_squared() * att_to_vec.length_squared());
    }
    
    vec_quat.q2 = vec_cross.x;
    vec_quat.q3 = vec_cross.y;
    vec_quat.q4 = vec_cross.z;
    vec_quat.normalize();
    
    Matrix3f ned_to_body;
    vec_quat.rotation_matrix(ned_to_body);
    
    thrust_decomp = ned_to_body * Vector3f(thrusts.x, thrusts.y, thrusts.z);
}

void Sub::thrust_decomposition_ned_rot_matrix(float* roll, float* pitch, 
    float* forward, float* lateral, float* throttle,
    bool is_do_log) {

    float roll_rad = ahrs.get_roll(), pitch_rad = ahrs.get_pitch();
    float yaw_rad = ahrs.get_yaw();

    Vector3f thrusts(*forward, *lateral, -(*throttle));
    Vector3f decomped;

    thrust_decomposition_att_error(ahrs.get_rotation_body_to_ned(), thrusts, decomped);

    *forward = decomped.x;
    *lateral = decomped.y;
    *throttle = -decomped.z;

    if (is_do_log) {
        AP::logger().Write("TDOF", "TimeUS,Roll,Pitch,Yaw,Forward_D, Forward, Lateral_D, Lateral, Throttle_D, Throttle", "Qffffff", 
                            AP_HAL::micros64(), 
                            (double)roll_rad, 
                            (double)pitch_rad,
                            (double)yaw_rad, 
                            (double)thrusts.x,
                            (double)decomped.x,
                            (double)thrusts.y,
                            (double)decomped.y,
                            (double)(-thrusts.z),
                            (double)(-decomped.z));
    }

    *roll = roll_rad;
    *pitch = pitch_rad;
}

void Sub::thrust_decomposition_body_rot_matrix(float* roll, float* pitch, 
    float* forward, float* lateral, float* throttle,
    bool is_do_log) {
    float roll_rad = ahrs.get_roll(), pitch_rad = ahrs.get_pitch();
    
    Vector3f thrusts(0, 0, -(*throttle));
    Vector3f decomped;

    thrust_decomposition_att_error(ahrs.get_rotation_body_to_ned(), thrusts, decomped);

    *forward += decomped.x;
    *lateral += decomped.y;
    *throttle = -decomped.z;

    *roll = roll_rad;
    *pitch = pitch_rad;
}

void Sub::thrust_decomposition_ned(float* roll, float* pitch, float* forward, float* lateral, float* throttle) {
    // _custom_thrust_factor can be derived from
    // forward thrust, front is +
    // lateral thrust, right is +
    // throttle thrust, up is +
    // so
    // if we need a NED throttle thrust named desired_throttle, it should be up or down
    //   throttle = desired_throttle * cos(pitch) * cos(roll)
    //   and compensate
    //   forward = desired_throttle * sin(pitch)
    //   lateral = -desired_throttle * sin(phi)
    // if we need a NED forward thrust named desired_forward, it should be front or back
    //   forward = desired_forward * cos(pitch)
    //   and compensate
    //   throttle = - desired_forward * sin(pitch)
    // if we need a NED lateral thrust named desired_lateral, it should be left or right
    //   lateral = desired_lateral * cos(phi)
    //   and compensate
    //   throttle = desired_lateral * sin(phi)

    float roll_rad = ahrs.get_roll(), pitch_rad = ahrs.get_pitch();

    float forward_thrust = *forward, lateral_thrust = *lateral, throttle_thrust = *throttle;
    float forward_tmp = *forward, lateral_tmp = *lateral;

    forward_thrust = forward_thrust * cosf(pitch_rad) + throttle_thrust * sinf(pitch_rad);
    lateral_thrust = lateral_thrust * cosf(roll_rad) - throttle_thrust * sinf(roll_rad);
    throttle_thrust = throttle_thrust * cosf(pitch_rad) * cosf(roll_rad) 
                    - forward_tmp * sinf(pitch_rad) 
                    + lateral_tmp * sinf(roll_rad);

    *forward = forward_thrust;
    *lateral = lateral_thrust;
    *throttle = throttle_thrust;

    *roll = roll_rad;
    *pitch = pitch_rad;
}

// throttle generated by z controller
void Sub::thrust_decomposition_alt_hold_body(float* roll, float* pitch, float* forward, float* lateral, float* throttle) {
    float roll_rad = ahrs.get_roll(), pitch_rad = ahrs.get_pitch();

    float forward_thrust = *forward, lateral_thrust = *lateral, throttle_thrust = *throttle;

    // throttle thurst need decomposition
    // forward and lateral not need
    
    forward_thrust += throttle_thrust * sinf(pitch_rad);
    lateral_thrust -= throttle_thrust * sinf(roll_rad);
    throttle_thrust = throttle_thrust * cosf(pitch_rad) * cosf(roll_rad);

    *forward = forward_thrust;
    *lateral = lateral_thrust;
    *throttle = throttle_thrust;

    *roll = roll_rad;
    *pitch = pitch_rad;
}

// throttle set by pilot
// no need to do decomposition

void Sub::thrust_decomposition_init(bool is_ned, control_mode_t mode) {
    if (mode != STABILIZE && mode != ALT_HOLD) {
        if (!is_ned_pilot_cleared) {
            thrust_decomposition_clear();
            is_ned_pilot_cleared = true;
            hal.shell->printf("decomposition cleard\r\n");
        }

        is_last_ned_pilot = is_ned;
        return;
    }

    if (is_last_ned_pilot == is_ned && !is_ned_pilot_cleared) {
        return;
    }

    is_last_ned_pilot = is_ned;
    is_ned_pilot_cleared = false;

    if (is_ned) {
        hal.shell->printf("set decomposition to NED\r\n");
        motors.set_thrust_decomposition_callback(
            FUNCTOR_BIND_MEMBER(&Sub::thrust_decomposition_ned_rot_matrix, void, float*, float*, float*, float*, float*, bool));
    } else {
        hal.shell->printf("set decomposition to body\r\n");
        motors.set_thrust_decomposition_callback(
            FUNCTOR_BIND_MEMBER(&Sub::thrust_decomposition_body_rot_matrix, void, float*, float*, float*, float*, float*, bool));
    }
}

void Sub::thrust_decomposition_clear() {
    motors.set_thrust_decomposition_callback(nullptr);
}

bool Sub::is_need_relax_z_controller(float forward, float lateral, float throttle) {
    Matrix3f body_to_ned = ahrs.get_rotation_body_to_ned();
    Vector3f thrusts(forward, lateral, -(throttle - 0.5f) * 2); // thorttle is 0 ~ 1 and body axis down is +
    Vector3f thrusts_ned = body_to_ned * thrusts;

    return fabsf(thrusts_ned.z) > 0.05f;
}
