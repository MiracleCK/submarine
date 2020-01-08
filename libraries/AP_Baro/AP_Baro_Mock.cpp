#include <AP_HAL/AP_HAL.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

#if defined(HAL_BARO_ALLOW_INIT_NO_BARO)

#include "AP_Baro_Mock.h"
#include "stdio.h"

extern const AP_HAL::HAL& hal;

/*
  constructor - registers instance at top Baro driver
 */
AP_Baro_Mock::AP_Baro_Mock(AP_Baro &baro) :
    AP_Baro_Backend(baro)
{
    _instance = _frontend.register_sensor();
#if APM_BUILD_TYPE(APM_BUILD_ArduSub)
        _frontend.set_type(_instance, AP_Baro::BARO_TYPE_WATER);
#endif
}

// Read the sensor
void AP_Baro_Mock::update(void)
{
    WITH_SEMAPHORE(_sem);
    _copy_to_frontend(_instance, 0, 0);
}

#endif  // CONFIG_HAL_BOARD
