#include "Sub.h"

// rtl_init - initialise guided controller
bool Sub::rtl_init(bool ignore_checks)
{
	hal.console->printf("rtl_init\n");
    if (!position_ok() && !ignore_checks) {
        return false;
    }

    // would not response to auto set home
    if (!ahrs.home_is_set() || !ahrs.home_is_locked()) {
        return false;
    }

    guided_init(true);
    guided_set_destination(ahrs.get_home());
   
    return true;
}

void Sub::rtl_run()
{
	guided_run();
}
