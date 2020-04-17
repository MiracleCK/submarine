//
// Unit tests for the AP_Math euler code
//

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

void setup();
void loop();
void test_matrix_rotate(void);
void test_frame_transforms(void);
void test_conversions(void);
void test_quaternion_eulers(void);
void test_matrix_eulers(void);

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#define SHOW_POLES_BREAKDOWN 0

static float rad_diff(float rad1, float rad2)
{
    float diff = rad1 - rad2;
    if (diff > M_PI) {
        diff -= 2 * M_PI;
    }
    if (diff < -M_PI) {
        diff += 2 * M_PI;
    }
    return fabsf(diff);
}

static void check_result(const char *msg,
                         float roll, float pitch, float yaw,
                         float roll2, float pitch2, float yaw2)
{
    if (isnan(roll2) ||
        isnan(pitch2) ||
        isnan(yaw2)) {
        hal.console->printf("%s NAN eulers roll=%f pitch=%f yaw=%f\n",
                            msg,
                            (double)roll,
                            (double)pitch,
                            (double)yaw);
    }

    if (rad_diff(roll2,roll) > ToRad(179)) {
        // reverse all 3
        roll2 += fmodf(roll2 + M_PI, 2 * M_PI);
        pitch2 += fmodf(pitch2 + M_PI, 2 * M_PI);
        yaw2 += fmodf(yaw2 + M_PI, 2 * M_PI);
    }

    if (rad_diff(roll2,roll) > 0.01f ||
        rad_diff(pitch2, pitch) > 0.01f ||
        rad_diff(yaw2, yaw) > 0.01f) {
        if (pitch >= M_PI/2 ||
            pitch <= -M_PI/2 ||
            ToDeg(rad_diff(pitch, M_PI/2)) < 1 ||
            ToDeg(rad_diff(pitch, -M_PI/2)) < 1) {
            // we expect breakdown at these poles
#if SHOW_POLES_BREAKDOWN
            hal.console->printf(
                "%s breakdown eulers roll=%f/%f pitch=%f/%f yaw=%f/%f\n",
                msg,
                (double)ToDeg(roll), (double)ToDeg(roll2),
                (double)ToDeg(pitch), (double)ToDeg(pitch2),
                (double)ToDeg(yaw), (double)ToDeg(yaw2));
#endif
        } else {
            hal.console->printf(
                "%s incorrect eulers roll=%f/%f pitch=%f/%f yaw=%f/%f\n",
                msg,
                (double)ToDeg(roll), (double)ToDeg(roll2),
                (double)ToDeg(pitch), (double)ToDeg(pitch2),
                (double)ToDeg(yaw), (double)ToDeg(yaw2));
        }
    }
}

static void test_euler(float roll, float pitch, float yaw)
{
    Matrix3f m;
    float roll2, pitch2, yaw2;

    m.from_euler(roll, pitch, yaw);
    m.to_euler(&roll2, &pitch2, &yaw2);
    check_result("test_euler", roll, pitch, yaw, roll2, pitch2, yaw2);
}

static const float angles[] = { 0, M_PI/8, M_PI/4, M_PI/2, M_PI,
                                -M_PI/8, -M_PI/4, -M_PI/2, -M_PI};

void test_matrix_eulers(void)
{
    uint8_t N = ARRAY_SIZE(angles);

    hal.console->printf("rotation matrix unit tests\n\n");

    for (uint8_t i = 0; i < N; i++)
        for (uint8_t j = 0; j < N; j++)
            for (uint8_t k = 0; k < N; k++)
                test_euler(angles[i], angles[j], angles[k]);

    hal.console->printf("tests done\n\n");
}

static void test_quaternion(float roll, float pitch, float yaw)
{
    Quaternion q;
    Matrix3f m;
    float roll2, pitch2, yaw2;

    q.from_euler(roll, pitch, yaw);
    q.to_euler(roll2, pitch2, yaw2);
    check_result("test_quaternion1", roll, pitch, yaw, roll2, pitch2, yaw2);

    m.from_euler(roll, pitch, yaw);
    m.to_euler(&roll2, &pitch2, &yaw2);
    check_result("test_quaternion2", roll, pitch, yaw, roll2, pitch2, yaw2);

    m.from_euler(roll, pitch, yaw);
    q.from_rotation_matrix(m);
    q.to_euler(roll2, pitch2, yaw2);
    check_result("test_quaternion3", roll, pitch, yaw, roll2, pitch2, yaw2);

    q.rotation_matrix(m);
    m.to_euler(&roll2, &pitch2, &yaw2);
    check_result("test_quaternion4", roll, pitch, yaw, roll2, pitch2, yaw2);
}

void test_quaternion_eulers(void)
{
    uint8_t N = ARRAY_SIZE(angles);

    hal.console->printf("quaternion unit tests\n\n");

    test_quaternion(M_PI/4, 0, 0);
    test_quaternion(0, M_PI/4, 0);
    test_quaternion(0, 0, M_PI/4);
    test_quaternion(-M_PI/4, 0, 0);
    test_quaternion(0, -M_PI/4, 0);
    test_quaternion(0, 0, -M_PI/4);
    test_quaternion(-M_PI/4, 1, 1);
    test_quaternion(1, -M_PI/4, 1);
    test_quaternion(1, 1, -M_PI/4);

    test_quaternion(ToRad(89), 0, 0.1f);
    test_quaternion(0, ToRad(89), 0.1f);
    test_quaternion(0.1f, 0, ToRad(89));

    test_quaternion(ToRad(91), 0, 0.1f);
    test_quaternion(0, ToRad(91), 0.1f);
    test_quaternion(0.1f, 0, ToRad(91));

    for (uint8_t i = 0; i < N; i++)
        for (uint8_t j = 0; j < N; j++)
            for (uint8_t k = 0; k < N; k++)
                test_quaternion(angles[i], angles[j], angles[k]);

    hal.console->printf("tests done\n\n");
}


static void test_conversion(float roll, float pitch, float yaw)
{
    Quaternion q;
    Matrix3f m, m2;

    float roll2, pitch2, yaw2;
    float roll3, pitch3, yaw3;

    q.from_euler(roll, pitch, yaw);
    q.to_euler(roll2, pitch2, yaw2);
    check_result("test_conversion1", roll, pitch, yaw, roll2, pitch2, yaw2);

    q.rotation_matrix(m);
    m.to_euler(&roll2, &pitch2, &yaw2);

    m2.from_euler(roll, pitch, yaw);
    m2.to_euler(&roll3, &pitch3, &yaw3);
    if (m.is_nan()) {
        hal.console->printf("NAN matrix roll=%f pitch=%f yaw=%f\n",
                            (double)roll,
                            (double)pitch,
                            (double)yaw);
    }

    check_result("test_conversion2", roll, pitch, yaw, roll2, pitch2, yaw2);
    check_result("test_conversion3", roll, pitch, yaw, roll3, pitch3, yaw3);
}

void test_conversions(void)
{
    uint8_t N = ARRAY_SIZE(angles);

    hal.console->printf("matrix/quaternion tests\n\n");

    test_conversion(1, 1.1f, 1.2f);
    test_conversion(1, -1.1f, 1.2f);
    test_conversion(1, -1.1f, -1.2f);
    test_conversion(-1, 1.1f, -1.2f);
    test_conversion(-1, 1.1f, 1.2f);

    for (uint8_t i = 0; i < N; i++)
        for (uint8_t j = 0; j < N; j++)
            for (uint8_t k = 0; k < N; k++)
                test_conversion(angles[i], angles[j], angles[k]);

    hal.console->printf("tests done\n\n");
}

void test_frame_transforms(void)
{
    Vector3f v, v2;
    Quaternion q;
    Matrix3f m;

    hal.console->printf("frame transform tests\n\n");

    q.from_euler(ToRad(45), ToRad(45), ToRad(45));
    q.normalize();
    m.from_euler(ToRad(45), ToRad(45), ToRad(45));

    v2 = v = Vector3f(0.0f, 0.0f, 1.0f);
    q.earth_to_body(v2);
    hal.console->printf("%f %f %f\n", (double)v2.x, (double)v2.y, (double)v2.z);
    v2 = m * v;
    hal.console->printf("%f %f %f\n\n", (double)v2.x, (double)v2.y, (double)v2.z);

    v2 = v = Vector3f(0.0f, 1.0f, 0.0f);
    q.earth_to_body(v2);
    hal.console->printf("%f %f %f\n", (double)v2.x, (double)v2.y, (double)v2.z);
    v2 = m * v;
    hal.console->printf("%f %f %f\n\n", (double)v2.x, (double)v2.y, (double)v2.z);

    v2 = v = Vector3f(1.0f, 0.0f, 0.0f);
    q.earth_to_body(v2);
    hal.console->printf("%f %f %f\n", (double)v2.x, (double)v2.y, (double)v2.z);
    v2 = m * v;
    hal.console->printf("%f %f %f\n", (double)v2.x, (double)v2.y, (double)v2.z);
}

// generate a random float between -1 and 1
static float rand_num(void)
{
    return ((2.0f * get_random16()) / 0xFFFF) - 1.0f;
}

void test_matrix_rotate(void)
{
    Matrix3f m1, m2, diff;
    Vector3f r;

    m1.identity();
    m2.identity();
    r.x = rand_num();
    r.y = rand_num();
    r.z = rand_num();

    for (uint16_t i = 0; i < 1000; i++) {
        // old method
        Matrix3f temp_matrix;
        temp_matrix.a.x = 0;
        temp_matrix.a.y = -r.z;
        temp_matrix.a.z =  r.y;
        temp_matrix.b.x =  r.z;
        temp_matrix.b.y = 0;
        temp_matrix.b.z = -r.x;
        temp_matrix.c.x = -r.y;
        temp_matrix.c.y =  r.x;
        temp_matrix.c.z = 0;
        temp_matrix = m1 * temp_matrix;
        m1 += temp_matrix;

        // new method
        m2.rotate(r);

        // check they behave in the same way
        diff = m1 - m2;
        float err = diff.a.length() + diff.b.length() + diff.c.length();

        if (err > 0) {
            hal.console->printf("ERROR: i=%u err=%f\n", (unsigned)i, (double)err);
        }
    }
}

void print_matrix_3f(const char* name, Matrix3f& matrix) {
    hal.console->printf("matrix %s\r\n", name);
    hal.console->printf("%f %f %f\r\n", matrix.a.x, matrix.a.y, matrix.a.z);
    hal.console->printf("%f %f %f\r\n", matrix.b.x, matrix.b.y, matrix.b.z);
    hal.console->printf("%f %f %f\r\n", matrix.c.x, matrix.c.y, matrix.c.z);
}

void thrust_decomposition(Vector3f euler, Vector3f thrust, Vector3f& thrust_decomp) {
    
    Matrix3f ned_to_body;
    ned_to_body.from_euler(euler.x, euler.y, euler.z);
    ned_to_body = ned_to_body.transposed();

    Matrix3f yaw_to_ned;
    float cy = cosf(euler.z);
    float sy = sinf(euler.z);
    yaw_to_ned.a.x = cy;
    yaw_to_ned.a.y = -sy;
    yaw_to_ned.a.z = 0;
    yaw_to_ned.b.x = sy;
    yaw_to_ned.b.y = cy;
    yaw_to_ned.b.z = 0;
    yaw_to_ned.c.x = 0;
    yaw_to_ned.c.y = 0;
    yaw_to_ned.c.z = 1;

    print_matrix_3f("yaw to ned", yaw_to_ned);
    print_matrix_3f("ned to body", ned_to_body);

    Vector3f tmp = Vector3f(thrust.x, thrust.y, -thrust.z);
    thrust_decomp = yaw_to_ned * tmp;
    hal.console->printf("thrust decomp after yaw to ned\r\n");
    hal.console->printf("%f %f %f\r\n", thrust_decomp.x, thrust_decomp.y, thrust_decomp.z);
    thrust_decomp = ned_to_body * thrust_decomp;

    thrust_decomp.z = -thrust_decomp.z;
}

void test_matrix_decomposition_r0p0y(void) {
    // roll 0 pitch 0 yaw ±45 ±90 ±135 ±180
    // thurst should not decomp
    hal.console->printf("\r\ntest roll 0° pitch 0° yaw changed\r\n");
    
    float yaw_array[] = {45, 90, 135, 180};
    Vector3f thrust = Vector3f(3, 4, 5);
    Vector3f thrust_decomp;

    for (uint8_t i = 0; i < sizeof(yaw_array) / sizeof(float); i++) {
        thrust_decomposition(Vector3f(0, 0, ToRad(yaw_array[i])), thrust, thrust_decomp);
        hal.console->printf("yaw %f decomp, [%f %f %f]\r\n", 
                yaw_array[i],
                thrust_decomp.x, thrust_decomp.y, thrust_decomp.z);
    }

    for (uint8_t i = 0; i < sizeof(yaw_array) / sizeof(float); i++) {
        thrust_decomposition(Vector3f(0, 0, ToRad(-yaw_array[i])), thrust, thrust_decomp);
        hal.console->printf("yaw %f decomp, [%f %f %f]\r\n", 
                -yaw_array[i],
                thrust_decomp.x, thrust_decomp.y, thrust_decomp.z);
    }
}

void test_matrix_decomposition_p0y0r(void) {
    // thurst should decomp
    // roll 45 
    //     forward not decomp
    //     lateral should be lateral and throttle
    //     throttle should be lateral and throttle
    // roll 90
    //     forward not decomp
    //     lateral should be -throttle
    //     throttle should be lateral
    // roll 180 
    //     forward not decomp
    //     lateral should be -lateral
    //     throttle should be -throttle
    hal.console->printf("\r\ntest pitch 0° yaw 0° roll changed\r\n");
    
    float roll_array[] = {45, 90, 135, 180};
    Vector3f thrust = Vector3f(3, 4, 5);
    Vector3f thrust_decomp;

    for (uint8_t i = 0; i < sizeof(roll_array) / sizeof(float); i++) {
        thrust_decomposition(Vector3f(ToRad(roll_array[i]), 0, 0), thrust, thrust_decomp);
        hal.console->printf("roll %f decomp, [%f %f %f]\r\n", 
                roll_array[i],
                thrust_decomp.x, thrust_decomp.y, thrust_decomp.z);
    }

    for (uint8_t i = 0; i < sizeof(roll_array) / sizeof(float); i++) {
        thrust_decomposition(Vector3f(ToRad(-roll_array[i]), 0, 0), thrust, thrust_decomp);
        hal.console->printf("roll %f decomp, [%f %f %f]\r\n", 
                -roll_array[i],
                thrust_decomp.x, thrust_decomp.y, thrust_decomp.z);
    }
}

void test_matrix_decomposition_r0y0p(void) {
    // thurst should decomp
    // pitch 45 
    //     forward should be forward and throttle
    //     lateral not decomp
    //     throttle should be forward and throttle
    // roll 90
    //     forward should be throttle
    //     lateral not decomp
    //     throttle should be -forward
    // roll 180 
    //     forward should be -forward
    //     lateral not decomp
    //     throttle should be -throttle
    hal.console->printf("\r\ntest pitch 0° yaw 0° roll changed\r\n");
    
    float pitch_array[] = {45, 90, 135, 180};
    Vector3f thrust = Vector3f(3, 4, 5);
    Vector3f thrust_decomp;

    for (uint8_t i = 0; i < sizeof(pitch_array) / sizeof(float); i++) {
        thrust_decomposition(Vector3f(0, ToRad(pitch_array[i]), 0), thrust, thrust_decomp);
        hal.console->printf("pitch %f decomp, [%f %f %f]\r\n", 
                pitch_array[i],
                thrust_decomp.x, thrust_decomp.y, thrust_decomp.z);
    }

    for (uint8_t i = 0; i < sizeof(pitch_array) / sizeof(float); i++) {
        thrust_decomposition(Vector3f(0, ToRad(pitch_array[i]), 0), thrust, thrust_decomp);
        hal.console->printf("pitch %f decomp, [%f %f %f]\r\n", 
                -pitch_array[i],
                thrust_decomp.x, thrust_decomp.y, thrust_decomp.z);
    }
}

void test_matrix_decomposition_real(void) {
    Vector3f thrust_decomp;
    thrust_decomposition(Vector3f(ToRad(153.2109), ToRad(88.8527), ToRad(-44.1103)),
        Vector3f(3, 4, 5), thrust_decomp);
    hal.console->printf("real angle decomp, [%f %f %f]\r\n", 
                thrust_decomp.x, thrust_decomp.y, thrust_decomp.z);

    thrust_decomposition(Vector3f(ToRad(3.3677), ToRad(88.8527), ToRad(166.1753)),
        Vector3f(3, 4, 5), thrust_decomp);
    hal.console->printf("old roll/yaw decomp, [%f %f %f]\r\n", 
                thrust_decomp.x, thrust_decomp.y, thrust_decomp.z);

    thrust_decomposition(Vector3f(ToRad(153.2109), ToRad(90), ToRad(-44.1103)),
        Vector3f(3, 4, 5), thrust_decomp);
    hal.console->printf("real roll/yaw pitch90 decomp, [%f %f %f]\r\n", 
                thrust_decomp.x, thrust_decomp.y, thrust_decomp.z);

    thrust_decomposition(Vector3f(ToRad(3.3677), ToRad(90), ToRad(166.1753)),
        Vector3f(3, 4, 5), thrust_decomp);
    hal.console->printf("old roll/yaw pitch90 decomp, [%f %f %f]\r\n", 
                thrust_decomp.x, thrust_decomp.y, thrust_decomp.z);
}

void test_matrix_decomposition(void) {
    hal.console->printf("matrix decomposition tests\r\n");

    // test_matrix_decomposition_r0p0y();
    // test_matrix_decomposition_p0y0r();
    // test_matrix_decomposition_r0y0p();
    test_matrix_decomposition_real();
    
    hal.console->printf("matrix decomposition test done \r\n");
}

/*
 *  euler angle tests
 */
void setup(void)
{
    hal.console->printf("euler unit tests\n\n");

    test_conversion(0, M_PI, 0);

    test_frame_transforms();
    test_conversions();
    test_quaternion_eulers();
    test_matrix_eulers();
    test_matrix_rotate();
    test_matrix_decomposition();
}

void loop(void) {}

AP_HAL_MAIN();
