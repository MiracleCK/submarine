
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

Vector3f Sub::thrust_decomposition_ned_rot_matrix(Vector3f& euler_rad, Vector3f thrusts) {

    euler_rad.x = ahrs.get_roll();
    euler_rad.y = ahrs.get_pitch();
    euler_rad.z = ahrs.get_yaw();

    Vector3f decomped;

    thrusts.z = -thrusts.z;

    thrust_decomposition_att_error(ahrs.get_rotation_body_to_ned(), thrusts, decomped);

    decomped.z = - decomped.z;

    return decomped;
}

Vector3f Sub::thrust_decomposition_body_rot_matrix(Vector3f& euler_rad, Vector3f thrusts) {
    euler_rad.x = ahrs.get_roll();
    euler_rad.y = ahrs.get_pitch();
    euler_rad.z = ahrs.get_yaw();
    
    Vector3f thrusts_decomp(0, 0, -thrusts.z);
    Vector3f decomped;

    thrust_decomposition_att_error(ahrs.get_rotation_body_to_ned(), thrusts_decomp, decomped);

    thrusts.x += decomped.x;
    thrusts.y += decomped.y;
    thrusts.z = -decomped.z;
    
    return thrusts;
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
            FUNCTOR_BIND_MEMBER(&Sub::thrust_decomposition_ned_rot_matrix, Vector3f, Vector3f&, Vector3f));
    } else {
        hal.shell->printf("set decomposition to body\r\n");
        motors.set_thrust_decomposition_callback(
            FUNCTOR_BIND_MEMBER(&Sub::thrust_decomposition_body_rot_matrix, Vector3f, Vector3f&, Vector3f));
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
