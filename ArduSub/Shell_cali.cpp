
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>

extern const AP_HAL::HAL& hal;

static bool calibrate_gyros();
static void calibrate_accel();
static void calibrate_accel_level();

char* cmd_cali_usage = "cali gyros|accel|baro [level]\r\n";

void cmd_cali(int argc, char *argv[]) {
    if (argc < 1) {
        hal.shell->printf(cmd_cali_usage);
        return;
    }

    if (!strcmp(argv[0], "gyros")) {
        calibrate_gyros();
        return;
    }

    if (!strcmp(argv[0], "accel")) {
        if (argc == 1) {
            calibrate_accel();
            return;
        } else if (!strcmp(argv[1], "level")) {
            calibrate_accel_level();
            return;
        }
    }

    if (!strcmp(argv[0], "baro")) {
        AP::baro().calibrate(true);
        hal.shell->printf("baro calibrate done\r\n");
        return;
    }

    hal.shell->printf(cmd_cali_usage);
}

bool calibrate_gyros() {
    AP::ins().init_gyro();
    if (!AP::ins().gyro_calibrated_ok_all()) {
        hal.shell->printf("gyro calirate failed\r\n");
        return false;
    }

    AP::ahrs().reset_gyro_drift();
    hal.shell->printf("gyro calbrate done\r\n");

    return true;
}

void calibrate_accel() {
    // start with gyro calibration
    if (!calibrate_gyros()) {
        return;
    }
    // start accel cal
    AP::ins().acal_init();
    AP::ins().get_acal()->start(NULL);

    hal.shell->printf("accel calibrate accepted\r\n");
}

void calibrate_accel_level() {
    if (!calibrate_gyros()) {
        return;
    }
    float trim_roll, trim_pitch;
    if (!AP::ins().calibrate_trim(trim_roll, trim_pitch)) {
        hal.shell->printf("accel level calibrate failed\r\n");
        return;
    }
    // reset ahrs's trim to suggested values from calibration routine
    AP::ahrs().set_trim(Vector3f(trim_roll, trim_pitch, 0));

    hal.shell->printf("accel level calbrate done\r\n");
}
