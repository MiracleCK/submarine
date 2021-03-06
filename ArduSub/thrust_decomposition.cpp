
#include "Sub.h"

static float calc_roll0_psi_from_rot(Matrix3f& rot, float &psi_o);
static Vector3f thrust_decomp_ned_roll0(Matrix3f &rot, Vector3f thrusts);
static Vector3f thrust_decomp_ned(Matrix3f &rot, Vector3f thrusts);

static void test_roll0_psi();
static void test_thrusts_decomp_roll0(Vector3f thrust, float theta_opts[], float psi_opts[], Vector3f decomped_opts[]);
static void test_decomp_roll0();
static void test_decomp_body();

float calc_roll0_psi_from_rot(Matrix3f& rot, float &psi_o) {
    // r: phi
    // p: theta
    // y: psi
    //
    // theta = M_PI_2
    //  -                              -      -               -
    // | 0  (srcy - crsy) (crcy + srsy) |    | 0 s(r-y) c(r-y) |
    // | 0  (srsy + crcy) (crsy - srcy) | -> | 0 c(r-y) s(y-r) |
    // |-1   0             0            |    |-1 0      0      |
    //  -                              -      -               -
    //
    // y - r = atan2(s(y-r) - s(r-y), c(r-y) + c(r-y))
    //
    // theta = -M_PI_2
    //  -                                -      -                 -
    // | 0  (-srcy - crsy) (-crcy + srsy) |    | 0 -s(r+y) -c(r+y) |
    // | 0  (-srsy + crcy) (-crsy - srcy) | -> | 0  c(r+y) -s(y+r) |
    // |-1    0              0            |    |-1  0       0      |
    //  -                                -      -                 -
    //
    // y + r = atan2(-s(y+r) -s(r+y), -c(r+y) - c(r+y))

    float theta = -safe_asin(rot.c.x);
    float psi;
    float phi = 0.0f;

    if (fabsf(theta - (float)M_PI_2) < 1.0e-3f) {
        psi = atan2f(rot.b.z - rot.a.y, rot.a.z + rot.b.y);
    } else if (fabsf(theta + (float)M_PI_2) < 1.0e-3f) {
        psi = atan2f(rot.b.z + rot.a.y, rot.a.z - rot.b.y);
    } else {
        phi = atan2f(rot.c.y, rot.c.z);
        psi = atan2f(rot.b.x, rot.a.x);
    }

    psi_o = psi;

    if (fabsf(phi - M_PI) < 1.0e-3f || fabsf(phi + M_PI) < 1.0e-3f) {
        // if (signbit(psi)) {
        //     psi += M_PI;
        // } else {
        //     psi -= M_PI;
        // }
    }

    return psi;
}

void test_roll0_psi() {
    float psi_opts[] = {0, 45, 90, 135, 180, -180, -135, -90, -45};
    float theta_opts[] = {0, 45, 90, 135, 180, -180, -135, -90, -45};
    float psi, psi_o;
    Matrix3f rot;
    for (int i = 0; i < (int)(sizeof(psi_opts)/sizeof(float)); i++) {
        for (int j = 0; j < (int)(sizeof(theta_opts)/sizeof(float)); j++) {
            rot.from_euler(ToRad(0), ToRad(theta_opts[j]), ToRad(psi_opts[i]));
            psi = calc_roll0_psi_from_rot(rot, psi_o);
            if (fabsf(psi - ToRad(psi_opts[i])) > 1.0e-3f) {
                printf("\r\ntest failed theta %f psi %f, calc psi_o %f psi %f\r\n", theta_opts[j], psi_opts[i], ToDeg(psi_o), ToDeg(psi));
                printf("rot a %f %f %f\r\n", rot.a.x, rot.a.y, rot.a.z);
                printf("rot b %f %f %f\r\n", rot.b.x, rot.b.y, rot.b.z);
                printf("rot c %f %f %f\r\n", rot.c.x, rot.c.y, rot.c.z);
            }
        }
    }
}

Vector3f thrust_decomp_ned_roll0(Matrix3f &rot, Vector3f thrusts) {
    // roll always zero
    // so wen need only do decomp at forward and throttle DOF
    // and b -> n rot can be simplified as
    //  -                                    -      -              -
    // | cpcy (srspcy - crsy) (crspcy + srsy) |    | cpcy -sy  spcy |
    // | cpsy (srspsy + crcy) (crspsy - srcy) | -> | cpsy  cy  spsy |
    // |-sp    srcp            crcp           |    |-sp    0   cp   |
    //  -                                    -      -              -
    // so we can got sy cy directly
    // float psi = calc_roll0_psi_from_rot(rot, psi_o);
    // Matrix3f yaw_rot(Vector3f(cosf(psi), -sinf(psi), 0), Vector3f(sinf(psi), cosf(psi), 0), Vector3f(0,0,1));
    Matrix3f yaw_rot(Vector3f(rot.b.y, rot.a.y, 0), Vector3f(-rot.a.y, rot.b.y, 0), Vector3f(0,0,1));
    
    thrusts.z = -thrusts.z;
    Vector3f decomoped = rot.transposed() * yaw_rot * thrusts;
    decomoped.z = -decomoped.z;

    return decomoped;
}

Vector3f thrust_decomp_ned(Matrix3f &rot, Vector3f thrusts) {
    thrusts.z = -thrusts.z;
    Vector3f decomoped = rot * thrusts;
    decomoped.z = -decomoped.z;

    return decomoped;
}

void test_thrusts_decomp_roll0(Vector3f thrust, float theta_opts[], float psi_opts[], Vector3f decomped_opts[]) {
    Vector3f decomped;
    Matrix3f rot;
    bool is_test_decomp_success;

    // forward
    is_test_decomp_success = true;
    // for (int i = 0; i < (int)(sizeof(psi_opts)/sizeof(float)); i++) {
    //     for (int j = 0; j < (int)(sizeof(theta_opts)/sizeof(float)); j++) {
    for (int i = 0; i < 9; i++) {
        for (int j = 0; j < 9; j++) {
            rot.from_euler(ToRad(0), ToRad(theta_opts[j]), ToRad(psi_opts[i]));
            decomped = thrust_decomp_ned_roll0(rot, thrust);
            if (fabsf(decomped_opts[j].x - decomped.x) > 1.0e-3f ||
                fabsf(decomped_opts[j].y - decomped.y) > 1.0e-3f ||
                fabsf(decomped_opts[j].z - decomped.z) > 1.0e-3f) {
                printf("failed: theta %f psi %f [%f %f %f] -> [%f %f %f]\r\n",
                    theta_opts[j], psi_opts[i],
                    thrust.x, thrust.y, thrust.z,
                    decomped.x, decomped.y, decomped.z);
                is_test_decomp_success = false;
            }
        }
    }
    if (is_test_decomp_success) {
        printf("success\r\n");
    }
}
void test_decomp_roll0() {
    float psi_opts[] = {0, 45, 90, 135, 180, -180, -135, -90, -45};
    float theta_opts[] = {0, 45, 90, 135, 180, -180, -135, -90, -45};
    Vector3f forward_decomp_thrusts(1.0f, 0.0f, 0.0f);
    Vector3f forward_decomped_opts[] = { // according to theta
        Vector3f(1.0f, 0.0f, 0.0f), // 0
        Vector3f(0.707107f, 0.0f, -0.707107f), // 45
        Vector3f(0.0f, 0.0f, -1.0f), // 90
        Vector3f(-0.707107f, 0.0f, -0.707107f), // 135
        Vector3f(-1.0f, 0.0f, 0.0f), // 180
        Vector3f(-1.0f, 0.0f, 0.0f), // -180
        Vector3f(-0.707107f, 0.0f, 0.707107f), // -135
        Vector3f(0.0f, 0.0f, 1.0f), // -90
        Vector3f(0.707107f, 0.0f, 0.707107f), // -45
    };
    Vector3f lateral_decomp_thrusts(0.0f, 1.0f, 0.0f);
    Vector3f lateral_decomped_opts[] = {
        Vector3f(0.0f, 1.0f, 0.0f), // 0
        Vector3f(0.0f, 1.0f, 0.0f), // 45
        Vector3f(0.0f, 1.0f, 0.0f), // 90
        Vector3f(0.0f, 1.0f, 0.0f), // 135
        Vector3f(0.0f, 1.0f, 0.0f), // 180
        Vector3f(0.0f, 1.0f, 0.0f), // -180
        Vector3f(0.0f, 1.0f, 0.0f), // -135
        Vector3f(0.0f, 1.0f, 0.0f), // -90
        Vector3f(0.0f, 1.0f, 0.0f), // -45
    };
    Vector3f throttle_decomp_thrusts(0.0f, 0.0f, 1.0f);
    Vector3f throttle_decomped_opts[] = {
        Vector3f(0.0f, 0.0f, 1.0f), // 0
        Vector3f(0.707107f, 0.0f, 0.707107f), // 45
        Vector3f(1.0f, 0.0f, 0.0f), // 90
        Vector3f(0.707107f, 0.0f, -0.707107f), // 135
        Vector3f(0.0f, 0.0f, -1.0f), // 180
        Vector3f(0.0f, 0.0f, -1.0f), // -180
        Vector3f(-0.707107f, 0.0f, -0.707107f), // -135
        Vector3f(-1.0f, 0.0f, 0.0f), // -90
        Vector3f(-0.707107f, 0.0f, 0.707107f), // -45
    };
    
    printf("\r\ntest forward thrust decomp\r\n");
    test_thrusts_decomp_roll0(forward_decomp_thrusts, theta_opts, psi_opts, forward_decomped_opts);

    printf("\r\ntest lateral thrust decomp\r\n");
    test_thrusts_decomp_roll0(lateral_decomp_thrusts, theta_opts, psi_opts, lateral_decomped_opts);

    printf("\r\ntest throttle thrust decomp\r\n");
    test_thrusts_decomp_roll0(throttle_decomp_thrusts, theta_opts, psi_opts, throttle_decomped_opts);
}

void test_decomp_body() {
    float phi_opts[] = {0, 45, 90, 135, 180, -180, -135, -90, -45};
    float psi_opts[] = {0, 45, 90, 135, 180, -180, -135, -90, -45};
    float theta_opts[] = {0, 45, 90, 135, 180, -180, -135, -90, -45};
    Vector3f decomped_angle;
    Vector3f decomped_matrix;
    Vector3f thrusts(0.0f, 0.0f, 1.0f);
    Vector3f decomp;
    Matrix3f rot;
    float phi, theta, psi;

    decomp = thrusts;
    decomp.z = -thrusts.z;

    bool is_test_success = true;
    
    for (int i = 0; i < (int)(sizeof(phi_opts)/sizeof(float)); i++) {
        for (int j = 0; j < (int)(sizeof(theta_opts)/sizeof(float)); j++) {
            for (int k = 0; k < (int)(sizeof(phi_opts)/sizeof(float)); k++) {
                phi = ToRad(phi_opts[i]);
                theta = ToRad(theta_opts[j]);
                psi = ToRad(psi_opts[k]);

                decomped_angle.x = thrusts.x * cosf(theta) + thrusts.z * sinf(theta);
                decomped_angle.y = thrusts.y * cosf(phi) 
                                 - thrusts.z * sinf(phi) * cosf(theta) 
                                 + thrusts.x * sinf(theta) * sinf(phi);
                decomped_angle.z = thrusts.z * cosf(theta) * cosf(phi) 
                                 - thrusts.x * sinf(theta) * cosf(phi)
                                 + thrusts.y * sinf(phi);

                rot.from_euler(phi, theta, psi);
                decomped_matrix = rot.transposed() * decomp;
                decomped_matrix.z = -decomped_matrix.z;

                if (fabsf(decomped_angle.x - decomped_matrix.x) > 1.0e-3f ||
                    fabsf(decomped_angle.y - decomped_matrix.y) > 1.0e-3f ||
                    fabsf(decomped_angle.z - decomped_matrix.z) > 1.0e-3f) {
                    printf("failed: phi %f theta %f psi %f, angle [%f %f %f] matrix [%f %f %f]\r\n",
                        phi_opts[i], theta_opts[j], psi_opts[k],
                        decomped_angle.x, decomped_angle.y, decomped_angle.z,
                        decomped_matrix.x, decomped_matrix.y, decomped_matrix.z);
                    is_test_success = false;
                }
            }
        }
    }

    if (is_test_success) {
        printf("sucess\r\n");
    }
}

Vector3f Sub::thrust_decomposition_ned_roll0(Vector3f& euler_rad, Vector3f thrusts, float throttle_bf) {
    euler_rad.x = ahrs.get_roll();
    euler_rad.y = ahrs.get_pitch();
    euler_rad.z = ahrs.get_yaw();

    Matrix3f rot = ahrs.get_rotation_body_to_ned();

    thrusts.z += throttle_bf;
    return thrust_decomp_ned_roll0(rot, thrusts);
}

Vector3f Sub::thrust_decomposition_ned(Vector3f& euler_rad, Vector3f thrusts, float throttle_bf) {
    euler_rad.x = ahrs.get_roll();
    euler_rad.y = ahrs.get_pitch();
    euler_rad.z = ahrs.get_yaw();
    // printf("Sub::thrust_decomposition_ned: degree roll %3.6f pitch %3.6f \r\n", ToDeg(euler_rad.x), ToDeg(euler_rad.y));

    Matrix3f rot = ahrs.get_rotation_body_to_ned();

    float cy = cosf(euler_rad.z);
    float sy = sinf(euler_rad.z);
    Matrix3f rot_yaw = Matrix3f(cy, -sy, 0, sy, cy, 0, 0, 0, 1);

    // printf("************Sub::thrust_decomposition_ned**********\r \n");
    // printf("************rot[]:**********\r \n");
    // for (size_t i = 0; i < 3; i++)
    // {
    //         printf("************%f %f %f**********\r \n",rot[i][0], rot[i][1], rot[i][2]);
    // }    
    rot = rot.transposed() * rot_yaw;
    // rot = Matrix3f(1, 0, 0, 0, 1, 0, 0, 0, 1);

    thrusts.z += throttle_bf;
    return thrust_decomp_ned(rot, thrusts);
}

Vector3f Sub::thrust_decomposition_body_rot_matrix(Vector3f& euler_rad, Vector3f thrusts, float throttle_bf) {
    euler_rad.x = ahrs.get_roll();
    euler_rad.y = ahrs.get_pitch();
    euler_rad.z = ahrs.get_yaw();

    // decomp thrust under body frame
    // only need rot matrix b -> n
    Matrix3f rot = ahrs.get_rotation_body_to_ned();

    Vector3f thrusts_decomp(0, 0, -thrusts.z);
    Vector3f decomped = rot.transposed() * thrusts_decomp;

    thrusts.x += decomped.x;
    thrusts.y += decomped.y;
    thrusts.z = throttle_bf - decomped.z;
    
    return thrusts;
}

// throttle set by pilot
// no need to do decomposition

void Sub::thrust_decomposition_select(bool is_ned, control_mode_t mode, bool is_affect_z_pos) {
    pilot_axis_t target_axis;

    if (mode != STABILIZE && mode != ALT_HOLD) {
        target_axis = AXIS_CLEARD;
    } else if (is_ned) {
        target_axis = AXIS_NED;
    } else {
        if (mode == ALT_HOLD) {
            if (is_affect_z_pos) {
                target_axis = AXIS_CLEARD;
            } else {
                target_axis = AXIS_BODY;
            }
        } else {
            target_axis = AXIS_CLEARD;
        }
    }

    if (pilot_axis == target_axis) {
        return;
    }

    if (target_axis == AXIS_NED) {
        printf("set decomposition to NED\r\n");
        motors.set_thrust_decomposition_callback(
            FUNCTOR_BIND_MEMBER(&Sub::thrust_decomposition_ned, Vector3f, Vector3f&, Vector3f, float));
        motors.set_motor_callback(
            FUNCTOR_BIND(&distance_control, &AC_DistanceControl::motor_monitor, void, int16_t *, uint8_t));
    } else if (target_axis == AXIS_BODY) {
        printf("set decomposition to body\r\n");
        motors.set_thrust_decomposition_callback(
            FUNCTOR_BIND_MEMBER(&Sub::thrust_decomposition_body_rot_matrix, Vector3f, Vector3f&, Vector3f, float));
		motors.set_motor_callback(nullptr);
    } else {
        printf("decomposition cleard\r\n");
        motors.set_thrust_decomposition_callback(nullptr);
        motors.set_motor_callback(nullptr);
        //motors.set_motor_callback(
        //    FUNCTOR_BIND(&distance_control, &AC_DistanceControl::motor_monitor, void, int16_t *, uint8_t));
    }

    pilot_axis = target_axis;
}

bool Sub::is_affect_z_pos(bool is_ned, float forward, float lateral, float throttle) {

	struct Location _current_loc;
    float depth; 
		
	ahrs.get_position(_current_loc);
	depth = _current_loc.alt * 0.01f;
	depth_limit = (depth < sub.g.depth_limit) ? 1 : 0;
    
    if (is_ned) {
    	if(depth_limit && throttle < 0.0f) {
			return 0;
    	}
        return (fabsf(throttle) > 0.05f);
    }

    Matrix3f body_to_ned = ahrs.get_rotation_body_to_ned();
    Vector3f thrusts(forward, lateral, -throttle); // body axis down is +
    Vector3f thrusts_ned = body_to_ned * thrusts;

    if(depth_limit && thrusts_ned.z > 0.0f) {
		return 0;
	}

    return fabsf(thrusts_ned.z) > 0.05f;
}
