#pragma once

#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AccelCalibrator.h"
#include "AP_Vehicle/AP_Vehicle_Type.h"

#define AP_ACCELCAL_MAX_NUM_CLIENTS 4
#define DETECT_ORIENTATION_SIDE_CNT 6

#define _send_pos(pos) do {                        \
    if (_gcs != nullptr) {                         \
        _gcs->send_accelcal_vehicle_position(pos); \
    }                                              \
} while(0)

#define _printf(fmt, args ...) do {                                 \
    if (_gcs != nullptr) {                                          \
        _gcs->send_text(MAV_SEVERITY_CRITICAL, fmt, ## args);       \
    }                                                               \
    hal.shell->printf(fmt "\r\n", ## args);                         \
} while (0)


class GCS_MAVLINK;
class AP_AccelCal_Client;

class AP_AccelCal {
public:
    AP_AccelCal():
    _use_gcs_snoop(true),
    _started(false),
    _saving(false)
    { update_status(); }

    // start all the registered calibrations
    void start(GCS_MAVLINK *gcs);

    // called on calibration cancellation
    void cancel();

    // Run an iteration of all registered calibrations
    void update();

    // get the status of the calibrator server as a whole
    accel_cal_status_t get_status() { return _status; }
    
    // Set vehicle position sent by the GCS
    bool gcs_vehicle_position(float position);

    // interface to the clients for registration
    static void register_client(AP_AccelCal_Client* client);

    void handleMessage(const mavlink_message_t &msg);

    // auto detect
    void auto_update();

private:
    GCS_MAVLINK *_gcs;
    bool _use_gcs_snoop;
    bool _waiting_for_mavlink_ack = false;
    uint32_t _last_position_request_ms;
    uint8_t _step;
    accel_cal_status_t _status;
    accel_cal_status_t _last_result;

    static uint8_t _num_clients;
    static AP_AccelCal_Client* _clients[AP_ACCELCAL_MAX_NUM_CLIENTS];

    // called on calibration success
    void success();

    // called on calibration failure
    void fail();

    // reset all the calibrators to there pre calibration stage so as to make them ready for next calibration request
    void clear();

    // proceed through the collection step for each of the registered calibrators
    void collect_sample();

    // update the state of the Accel calibrator server
    void update_status();

    // checks if no new sample has been received for considerable amount of time
    bool check_for_timeout();

    // check if client's calibrator is active
    bool client_active(uint8_t client_num);

    bool _started;
    bool _saving;

    uint8_t _num_active_calibrators;

    AccelCalibrator* get_calibrator(uint8_t i);

    // auto detect cal
    // according to ACCELCAL_VEHICLE_POS order
    // but start from 0
    enum detect_orientation {
        DETECT_ORIENTATION_LEVEL,
        DETECT_ORIENTATION_LEFT,
        DETECT_ORIENTATION_RIGHT,
        DETECT_ORIENTATION_NOSE_DOWN,
        DETECT_ORIENTATION_NOSE_UP,
        DETECT_ORIENTATION_BACK,
        DETECT_ORIENTATION_ERROR,
        DETECT_ORIENTATION_DETECTING
    };

    struct detect_orientation_s{
        uint64_t t_start;
        uint64_t t_prev;
        uint64_t t_still;
        uint64_t t_timeout;

        float accel_ema[3]; // exponential moving average of accel
        float accel_disp[3]; // max-hold dispersion of accel

        bool is_inited;

        detect_orientation orientation;
    };

    detect_orientation_s detect;

    void _detect_init(void);
    detect_orientation detect_orientation_auto(void);
};

class AP_AccelCal_Client {
friend class AP_AccelCal;
private:
    // getters
    virtual bool _acal_get_saving() { return false; }
    virtual bool _acal_get_ready_to_sample() { return true; }
    virtual bool _acal_get_fail() { return false; }
    virtual AccelCalibrator* _acal_get_calibrator(uint8_t instance) = 0;

    // events
    virtual void _acal_save_calibrations() = 0;
    virtual void _acal_event_success() {};
    virtual void _acal_event_cancellation() {};
    virtual void _acal_event_failure() {};
};
