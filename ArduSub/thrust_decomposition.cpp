
#include "Sub.h"

void Sub::thrust_decomposition_ned(float* forward, float* lateral, float* throttle) {
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

    float roll = ahrs.get_roll(), pitch = ahrs.get_pitch();

    float forward_thrust = *forward, lateral_thrust = *lateral, throttle_thrust = *throttle;
    float forward_tmp = *forward, lateral_tmp = *lateral;

    forward_thrust = forward_thrust * cosf(pitch) + throttle_thrust * sinf(pitch);
    lateral_thrust = lateral_thrust * cosf(roll) - throttle_thrust * sinf(roll);
    throttle_thrust = throttle_thrust * cosf(pitch) * cosf(roll) 
                    - forward_tmp * sinf(pitch) 
                    + lateral_tmp * sinf(roll);

    *forward = forward_thrust;
    *lateral = lateral_thrust;
    *throttle = throttle_thrust;
}

// throttle generated by z controller
void Sub::thrust_decomposition_alt_hold_body(float* forward, float* lateral, float* throttle) {
    float roll = ahrs.get_roll(), pitch = ahrs.get_pitch();

    float forward_thrust = *forward, lateral_thrust = *lateral, throttle_thrust = *throttle;

    // throttle thurst need decomposition
    // forward and lateral not need

    forward_thrust += throttle_thrust * sinf(pitch);
    lateral_thrust -= throttle_thrust * sinf(roll);
    throttle_thrust = throttle_thrust * cosf(pitch) * cosf(roll);

    *forward = forward_thrust;
    *lateral = lateral_thrust;
    *throttle = throttle_thrust;
}

// throttle set by pilot
// no need to do decomposition

void Sub::thrust_decomposition_init(bool is_ned, control_mode_t mode) {
    if (mode == MANUAL) {
        thrust_decomposition_clear();
        return;
    }

    if (is_ned) {
        motors.set_thrust_decomposition_callback(
            FUNCTOR_BIND_MEMBER(&Sub::thrust_decomposition_ned, void, float*, float*, float*));
    } else {
        motors.set_thrust_decomposition_callback(
            FUNCTOR_BIND_MEMBER(&Sub::thrust_decomposition_alt_hold_body, void, float*, float*, float*));
    }
}

void Sub::thrust_decomposition_clear() {
    motors.set_thrust_decomposition_callback(nullptr);
}