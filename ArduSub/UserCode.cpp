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
    static Vector2f count;
    
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

    if(count.x > 5 || count.y > 5) {
		gcs().send_text(MAV_SEVERITY_WARNING, "May hit an obstacle");
    }
}
#endif
