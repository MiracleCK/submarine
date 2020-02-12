
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_AccelCal.h"

#define CONSTANTS_ONE_G   9.80665f
#define AP_ACCELCAL_AUTO_POSITION_REQUEST_INTERVAL_MS 1000

#define _mix_printf(fmt, args ...) do {                                 \
        if (_gcs != nullptr) {                                          \
            _gcs->send_text(MAV_SEVERITY_CRITICAL, fmt, ## args);       \
        }                                                               \
        hal.shell->printf(fmt "\r\n", ## args);                        \
    } while (0)

const extern AP_HAL::HAL& hal;

const char* orientation_name[] = {
    "level", "left", "right", "nose down", "nose up", "back",
};

void AP_AccelCal::auto_update()
{
    if (!get_calibrator(0)) {
        // no calibrators
        return;
    }

    if (_started) {
        update_status();

        AccelCalibrator *cal;
        uint8_t num_active_calibrators = 0;
        for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
            num_active_calibrators++;
        }
        if (num_active_calibrators != _num_active_calibrators) {
            fail();
            return;
        }
        
        switch(_status) {
            case ACCEL_CAL_NOT_STARTED:
                fail();
                return;
            case ACCEL_CAL_WAITING_FOR_ORIENTATION: {
                // if we're waiting for orientation, first ensure that all calibrators are on the same step
                detect_orientation orientation;
                if ((cal = get_calibrator(0)) == NULL) {
                    fail();
                    return;
                }

                uint8_t num_samples_collected = cal->get_num_samples_collected();

                if (!detect.is_inited) {
                    _detect_init();

                    if (num_samples_collected < sizeof(orientation_name)){
                        _mix_printf("Pending: %s", orientation_name[num_samples_collected]);

                        uint32_t now = AP_HAL::millis();
                        if (now - _last_position_request_ms > AP_ACCELCAL_AUTO_POSITION_REQUEST_INTERVAL_MS) {
                            _last_position_request_ms = now;
                            _gcs->send_accelcal_vehicle_position(num_samples_collected+1);
                        }
                    } else {
                        fail();
                    }

                    return;
                }
                
                orientation = detect_orientation_auto();
                if (orientation == DETECT_ORIENTATION_DETECTING){
                }else if (orientation == DETECT_ORIENTATION_ERROR){
                    fail();
                }else{
                    // need as level/left/right/down/up/back order
                    if (orientation == num_samples_collected) {
                        detect.orientation = orientation;
                        collect_sample();
                    }

                    detect.is_inited = false;
                }
                break;
            }
            case ACCEL_CAL_COLLECTING_SAMPLE:
                // check for timeout

                for(uint8_t i=0; (cal = get_calibrator(i)); i++) {
                    cal->check_for_timeout();
                }

                update_status();

                if (_status == ACCEL_CAL_FAILED) {
                    fail();
                }
                return;
            case ACCEL_CAL_SUCCESS:
                // save
                if (_saving) {
                    bool done = true;
                    for(uint8_t i=0; i<_num_clients; i++) {
                        if (client_active(i) && _clients[i]->_acal_get_saving()) {
                            done = false;
                            break;
                        }
                    }
                    if (done) {
                        success();
                    }
                    return;
                } else {
                    for(uint8_t i=0; i<_num_clients; i++) {
                        if(client_active(i) && _clients[i]->_acal_get_fail()) {
                            fail();
                            return;
                        }
                    }
                    for(uint8_t i=0; i<_num_clients; i++) {
                        if(client_active(i)) {
                            _clients[i]->_acal_save_calibrations();
                        }
                    }
                    _saving = true;
                }
                return;
            default:
            case ACCEL_CAL_FAILED:
                fail();
                return;
        }
    } else if (_last_result != ACCEL_CAL_NOT_STARTED) {
        // only continuously report if we have ever completed a calibration
        uint32_t now = AP_HAL::millis();
        if (now - _last_position_request_ms > AP_ACCELCAL_AUTO_POSITION_REQUEST_INTERVAL_MS) {
            _last_position_request_ms = now;
            switch (_last_result) {
                case ACCEL_CAL_SUCCESS:
                    _gcs->send_accelcal_vehicle_position(ACCELCAL_VEHICLE_POS_SUCCESS);
                    break;
                case ACCEL_CAL_FAILED:
                    _gcs->send_accelcal_vehicle_position(ACCELCAL_VEHICLE_POS_FAILED);
                    break;
                default:
                    // should never hit this state
                    break;
            }
        }
    }
}

AP_AccelCal::detect_orientation AP_AccelCal::detect_orientation_auto(void)
{
    const unsigned ndim = 3;

    bool lenient_still_position = false;
    float* accel_ema = &detect.accel_ema[0];
    float* accel_disp = &detect.accel_disp[0];
    float       ema_len = 0.5f;             // EMA time constant in seconds
    const float normal_still_thr = 0.25;        // normal still threshold
    float       still_thr2 = powf(lenient_still_position ? (normal_still_thr * 3) : normal_still_thr, 2);
    float       accel_err_thr = 5.0f;           // set accel error threshold to 5m/s^2
    uint64_t still_time = lenient_still_position ? 1000000 : 1500000;    // still time required in us

    uint64_t t;
    /* set timeout to 30s */
    uint64_t timeout = 30000000;
    uint64_t* t_timeout = &detect.t_timeout;
    uint64_t* t_prev = &detect.t_prev;
    uint64_t* t_still = &detect.t_still;

    Vector3f samp;
    AP_InertialSensor *sensor = (AP_InertialSensor*)_clients[0];

    //printf("detecting orientation ...\r\n");

    do {

        samp = sensor->get_accel(0);
        t = AP_HAL::micros64();
        float dt = (t - (*t_prev)) / 1000000.0f;
        *t_prev = t;
        float w = dt / ema_len;

        for (unsigned i = 0; i < ndim; i++) {

            float di = 0.0f;
            switch (i) {
                case 0:
                    di = samp.x;
                    break;
                case 1:
                    di = samp.y;
                    break;
                case 2:
                    di = samp.z;
                    break;
            }

            float d = di - accel_ema[i];
            accel_ema[i] += d * w;
            d = d * d;
            accel_disp[i] = accel_disp[i] * (1.0f - w);

            if (d > still_thr2 * 8.0f) {
                d = still_thr2 * 8.0f;
            }

            if (d > accel_disp[i]) {
                accel_disp[i] = d;
            }
        }

        /* still detector with hysteresis */
        if (accel_disp[0] < still_thr2 &&
            accel_disp[1] < still_thr2 &&
            accel_disp[2] < still_thr2) {
            /* is still now */
            if (*t_still == 0) {
                /* first time */
                _mix_printf("detecting, hold still...");
                
                
                *t_still = t;
                *t_timeout = t + timeout;
            } else {
                /* still since t_still */
                if (t > *t_still + still_time) {
                    /* vehicle is still, exit from the loop to detection of its orientation */
                    break;
                }
            }
        } else if (accel_disp[0] > still_thr2 * 4.0f ||
               accel_disp[1] > still_thr2 * 4.0f ||
               accel_disp[2] > still_thr2 * 4.0f) {
            /* not still, reset still start time */
            if (*t_still != 0) {
                _mix_printf("detected motion, hold still...");

                hal.scheduler->delay(500);
                *t_still = 0;
            }
        }

        if (t > *t_timeout) {
            _mix_printf("detect timeout");
            return DETECT_ORIENTATION_ERROR;
        }

        return DETECT_ORIENTATION_DETECTING;
    }while(0);

    if (fabsf(accel_ema[0] - CONSTANTS_ONE_G) < accel_err_thr &&
        fabsf(accel_ema[1]) < accel_err_thr &&
        fabsf(accel_ema[2]) < accel_err_thr) {
        _mix_printf("detected nose up");
        return DETECT_ORIENTATION_NOSE_UP;        // [ g, 0, 0 ]
    }

    if (fabsf(accel_ema[0] + CONSTANTS_ONE_G) < accel_err_thr &&
        fabsf(accel_ema[1]) < accel_err_thr &&
        fabsf(accel_ema[2]) < accel_err_thr) {
        _mix_printf("detected nose down");
        return DETECT_ORIENTATION_NOSE_DOWN;        // [ -g, 0, 0 ]
    }

    if (fabsf(accel_ema[0]) < accel_err_thr &&
        fabsf(accel_ema[1] - CONSTANTS_ONE_G) < accel_err_thr &&
        fabsf(accel_ema[2]) < accel_err_thr) {
        _mix_printf("detected left");
        return DETECT_ORIENTATION_LEFT;        // [ 0, g, 0 ]
    }

    if (fabsf(accel_ema[0]) < accel_err_thr &&
        fabsf(accel_ema[1] + CONSTANTS_ONE_G) < accel_err_thr &&
        fabsf(accel_ema[2]) < accel_err_thr) {
        _mix_printf("detected right");
        return DETECT_ORIENTATION_RIGHT;        // [ 0, -g, 0 ]
    }

    if (fabsf(accel_ema[0]) < accel_err_thr &&
        fabsf(accel_ema[1]) < accel_err_thr &&
        fabsf(accel_ema[2] - CONSTANTS_ONE_G) < accel_err_thr) {
        _mix_printf("detected back");
        return DETECT_ORIENTATION_BACK;        // [ 0, 0, g ]
    }

    if (fabsf(accel_ema[0]) < accel_err_thr &&
        fabsf(accel_ema[1]) < accel_err_thr &&
        fabsf(accel_ema[2] + CONSTANTS_ONE_G) < accel_err_thr) {
        _mix_printf("detected level");
        return DETECT_ORIENTATION_LEVEL;        // [ 0, 0, -g ]
    }

    _mix_printf("ERROR: invalid orientation");

    return DETECT_ORIENTATION_ERROR;    // Can't detect orientation
}

void AP_AccelCal::_detect_init(void) {
    detect.accel_ema[0] = 0.0f;
    detect.accel_ema[1] = 0.0f;
    detect.accel_ema[2] = 0.0f;

    detect.accel_disp[0] = 0.0f;
    detect.accel_disp[1] = 0.0f;
    detect.accel_disp[2] = 0.0f;

    uint64_t t_start = AP_HAL::micros64();
    detect.t_prev = t_start;
    detect.t_still = 0;
    detect.t_timeout = t_start + 30000000; // 30s

    detect.is_inited = true;
}

