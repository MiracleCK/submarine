#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include <Filter/LowPassFilter2p.h>

// Data timeout
#define AP_RANGEFINDER_MAVLINK_TIMEOUT_MS 1000 //500

class AP_RangeFinder_MAVLink : public AP_RangeFinder_Backend
{

public:
    // constructor
    AP_RangeFinder_MAVLink(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params);

    // static detection function
    static bool detect();

    // update state
    void update(void) override;

    // Get update from mavlink
    void handle_msg(const mavlink_message_t &msg) override;

    bool distance_ok(float distance);

protected:

    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return sensor_type;
    }

private:
    uint16_t distance_cm;
	uint16_t distance_cm_filtered;
    LowPassFilter2pInt _distance_filter;
    int16_t sample_freq;
    int16_t cutoff_freq;
    float _mean_distance;
    uint32_t _error_count;
    // start a reading
    static bool start_reading(void);
    static bool get_reading(uint16_t &reading_cm);

    MAV_DISTANCE_SENSOR sensor_type = MAV_DISTANCE_SENSOR_UNKNOWN;
};
