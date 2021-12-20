#include "CH_WaterDetector.h"
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
#include <hwdef.h>
#endif

extern const AP_HAL::HAL &hal;

CH_WaterDetector::CH_WaterDetector(AP_HAL::AnalogSource *a1, AP_HAL::AnalogSource *a2)
{
    analog1 = a1;
    analog2 = a2;
}

void CH_WaterDetector::init()
{
    state = 3;
    ain1 = 0;
    ain2 = 0;
}

void CH_WaterDetector::update()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    uint8_t st;
    ain1 = analog1->read_average();
    ain2 = analog2->read_average();
    if(ain2 < 3500)
        st = 0;
    else
        st = 2;
    if(ain1 >= 3500)
        st |= 1;

    if (state ^ st)
    {
        state = st;
    }
#else
    state = 0;
#endif
}
