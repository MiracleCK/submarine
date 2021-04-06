#include "Sub.h"

#ifdef USERHOOK_INIT
void Sub::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
}
#endif

#ifdef USERHOOK_FASTLOOP
void Sub::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Sub::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Sub::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Sub::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Sub::userhook_SuperSlowLoop()
{
    // put your 1Hz code here  
    static Vector3i count;
    
    const Vector3f& curr_vel = inertial_nav.get_velocity();
    if(fabsf(motors.get_forward()) > 0.5f && fabsf(curr_vel.x) < 8.0f) {
		count.x++;
    } else {
		count.x = 0;
    }

    if(fabsf(motors.get_lateral()) > 0.5f && fabsf(curr_vel.y) < 8.0f) {
		count.y++;
    } else {
		count.y = 0;
    }

	static Vector3f last_pos;
	Vector3f curr_pos, err_pos;
	if(control_mode==POSHOLD) {
	    curr_pos = inertial_nav.get_position();
	    if(curr_pos.is_zero()) {
			last_pos = curr_pos;
			err_pos.zero();
	    } else {
			err_pos =  curr_pos - last_pos;
			last_pos = curr_pos;
	    }

	    float error_length = norm(err_pos.x, err_pos.y);
	    if((fabsf(motors.get_forward()) > 0.5f || 
	       fabsf(motors.get_lateral()) > 0.5f) && 
	       error_length < 5.0f) {
			count.z++;
	    } else {
			count.z = 0;
	    }
	    //hal.shell->printf("error_length %.04f, count.z %d\r\n", error_length, count.z);
    } else {
		last_pos.zero();
    }

    if(count.x > 5 || count.y > 5 || count.z > 5) {
		gcs().send_text(MAV_SEVERITY_WARNING, "May hit an obstacle");
		hal.shell->printf("[%d %d %d]May hit an obstacle\r\n", count.x > 0, count.y > 0, count.z > 0);
    }
}
#endif
