/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_RangeFinder_MAVLink.h"
#include <AP_HAL/AP_HAL.h>



extern const AP_HAL::HAL& hal;

/*
   The constructor also initialises the rangefinder. Note that this
   constructor is not called until detect() returns true, so we
   already know that we should setup the rangefinder
*/
AP_RangeFinder_MAVLink::AP_RangeFinder_MAVLink(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
    state.last_reading_ms = AP_HAL::millis();
    distance_cm = 0;
    sample_freq = params.sample_freq.get();
    cutoff_freq = params.cutoff_freq.get();
    //_distance_filter.set_cutoff_frequency((float)sample_freq, (float)cutoff_freq);
}

/*
   detect if a MAVLink rangefinder is connected. We'll detect by
   checking a parameter.
*/
bool AP_RangeFinder_MAVLink::detect()
{
    // Assume that if the user set the RANGEFINDER_TYPE parameter to MAVLink,
    // there is an attached MAVLink rangefinder
    return true;
}

//static constexpr float FILTER_KOEF = 0.1f;

/* Check that the baro value is valid by using a mean filter. If the
 * value is further than filtrer_range from mean value, it is
 * rejected. 
*/
bool AP_RangeFinder_MAVLink::distance_ok(float distance)
{
    if (isinf(distance) || isnan(distance) || distance > 600.0f || distance < 5) {
    //if (isinf(distance) || isnan(distance)) {
        return false;
    }

    const float _range = params.range.get();
    if (_range <= 0) {
        return true;
    }

    bool ret = true;
    if (is_zero(_mean_distance)) {
        _mean_distance = distance;
    } else {
        const float d = fabsf(_mean_distance - distance) / (_mean_distance + distance);  // diff divide by mean value in percent ( with the * 200.0f on later line)
        float koeff = params.koef.get();//FILTER_KOEF;

        if (d * 200.0f > _range) {  // check the difference from mean value outside allowed range
            //hal.shell->printf("\r\ndistance error: mean %f got %f\r\n", (double)_mean_distance, (double)distance );
            ret = false;
            koeff /= (d * 10.0f);  // 2.5 and more, so one bad sample never change mean more than 4%
            _error_count++;
        }

        if(0 && (Rotation)params.orientation.get() == ROTATION_NONE) {
			static uint32_t _startup_ms = 0;

			if(_startup_ms == 0) {
				_startup_ms = AP_HAL::millis();
			}

			if(AP_HAL::millis() - _startup_ms > 1000) {
				_startup_ms = AP_HAL::millis();
				
				hal.shell->printf("%.04f %.04f %.04f %.04f\r\n", 
							_mean_distance,
							distance, 
							d,
							koeff);
			}
		}
        _mean_distance = _mean_distance * (1 - koeff) + distance * koeff; // complimentary filter 1/k
    }
    return ret;
}


/*
   Set the distance based on a MAVLINK message
*/
void AP_RangeFinder_MAVLink::handle_msg(const mavlink_message_t &msg)
{
    mavlink_distance_sensor_t packet;
    mavlink_msg_distance_sensor_decode(&msg, &packet);

    // only accept distances for downward facing sensors
    //if (packet.orientation == MAV_SENSOR_ROTATION_PITCH_270) {
    if (packet.orientation == (Rotation)params.orientation.get()) {
    	//if(packet.current_distance > 600 || packet.current_distance < 5) {
		//	return ;
    	//}
    	
    	//hal.shell->printf("orientation %d, ms %d\r\n", (int)packet.orientation, AP_HAL::millis() - state.last_reading_ms);
    	state.last_reading_ms = AP_HAL::millis();
        distance_cm = packet.current_distance;
        sensor_type = (MAV_DISTANCE_SENSOR)packet.type;  
        //distance_cm_filtered = _distance_filter.apply(distance_cm);
        //hal.shell->printf("orientation %d, distance_cm %d\r\n", (int)packet.orientation, distance_cm);
        
        if(distance_ok((float)distance_cm)) {
	    	distance_cm_filtered = distance_cm;
	    	//hal.shell->printf("orientation %d, distance_cm_filtered %d\r\n", (int)packet.orientation, distance_cm_filtered);
	    }
    }
}

/*
   update the state of the sensor
*/
void AP_RangeFinder_MAVLink::update(void)
{
    //Time out on incoming data; if we don't get new
    //data in 500ms, dump it
#if 0
    if (AP_HAL::millis() - state.last_reading_ms > AP_RANGEFINDER_MAVLINK_TIMEOUT_MS) {
        set_status(RangeFinder::Status::NoData);
        state.distance_cm = 0;
    } else {
        state.distance_cm = distance_cm;
        //state.distance_cm_filtered = distance_cm_filtered;
        //state.distance_cm_filtered = _distance_filter.apply(distance_cm);
        update_status();
    }

    if(distance_ok((float)state.distance_cm)) {
    	state.distance_cm_filtered = state.distance_cm;
    }
#endif

	state.distance_cm = distance_cm;
	state.distance_cm_filtered = distance_cm_filtered;
	//state.distance_cm_filtered = _distance_filter.apply(distance_cm);

	//if(distance_ok((float)state.distance_cm)) {
    //	state.distance_cm_filtered = state.distance_cm;
    //}
    
	set_status(RangeFinder::Status::Good);

    if((sample_freq != params.sample_freq.get()) ||
       (cutoff_freq != params.cutoff_freq.get())) {
		sample_freq = params.sample_freq.get();
	    cutoff_freq = params.cutoff_freq.get();
	    hal.shell->printf("sample_freq: %d, cutoff_freq: %d\r\n", sample_freq, cutoff_freq);
	    //_distance_filter.set_cutoff_frequency((float)sample_freq, (float)cutoff_freq);
    }
}
