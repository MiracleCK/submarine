#include "CH_WaterDetector.h"
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
#include <hwdef.h>
#endif

extern const AP_HAL::HAL &hal;

static uint8_t _state = 0;

void CH_WaterDetector::init()
{
    _state = 3;
}

void CH_WaterDetector::update()
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
#if defined(HAL_GPIO_PIN_WATER_DETECTOR1) && defined(HAL_GPIO_PIN_WATER_DETECTOR2)
    uint8_t s = palReadLine(HAL_GPIO_PIN_WATER_DETECTOR1);
    _state = s|(palReadLine(HAL_GPIO_PIN_WATER_DETECTOR2)<<1);
#endif
#endif
}

uint8_t CH_WaterDetector::read(void)
{
    return _state;
}
