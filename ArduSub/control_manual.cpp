#include "Sub.h"

// manual_init - initialise manual controller
bool Sub::manual_init()
{
    // set target altitude to zero for reporting
    pos_control.set_alt_target(0);

    // attitude hold inputs become thrust inputs in manual mode
    // set to neutral to prevent chaotic behavior (esp. roll/pitch)
    set_neutral_controls();

    return true;
}

// manual_run - runs the manual (passthrough) controller
// should be called at 100hz or more
void Sub::manual_run()
{
    if (smart_mode_auto_switch()) {
        return;
    }
    
    // if not armed set throttle to zero and exit immediately
    if (!motors.armed()) {
        motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::GROUND_IDLE);
        attitude_control.set_throttle_out(0,true,g.throttle_filt);
        attitude_control.relax_attitude_controllers();
        return;
    }

    motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    motors.set_roll(channel_roll->norm_input());
    motors.set_pitch(channel_pitch->norm_input());
    motors.set_yaw(channel_yaw->norm_input() * g.acro_yaw_p / ACRO_YAW_P);
    motors.set_throttle(channel_throttle->norm_input());
    motors.set_forward(channel_forward->norm_input());
    motors.set_lateral(channel_lateral->norm_input());

    if(1) {
        static uint32_t _startup_ms = 0;

        if(_startup_ms == 0) {
			_startup_ms = AP_HAL::millis();
        }

        if(AP_HAL::millis() - _startup_ms > 1000) {
			_startup_ms = AP_HAL::millis();
			
		    printf("roll %d\r\n", channel_roll->get_radio_in());
		    printf("pitch %d\r\n", channel_pitch->get_radio_in());
		    printf("yaw %d\r\n", channel_yaw->get_radio_in());
		    printf("throttle %d\r\n", channel_throttle->get_radio_in());
		    printf("forward %d\r\n", channel_forward->get_radio_in());
		    printf("lateral %d\r\n", channel_lateral->get_radio_in());
		    printf("\r\n");
		    printf("roll %d\r\n", RC_IN_CHANNEL_ROLL);
		    printf("pitch %d\r\n", RC_IN_CHANNEL_PITCH);
		    printf("yaw %d\r\n", RC_IN_CHANNEL_YAW);
		    printf("throttle %d\r\n", RC_IN_CHANNEL_THROTTLE);
		    printf("forward %d\r\n", RC_IN_CHANNEL_FORWARD);
		    printf("lateral %d\r\n", RC_IN_CHANNEL_LATERAL);
		    printf("\r\n");
	    }
	}
}
