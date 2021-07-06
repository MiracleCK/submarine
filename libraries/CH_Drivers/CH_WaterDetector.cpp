#include "CH_WaterDetector.h"
#include <AP_HAL/AP_HAL.h>
#include <hwdef.h>

extern const AP_HAL::HAL &hal;

static uint8_t _state = 0;

void CH_WaterDetector::init()
{
    _state = 0;
}

void CH_WaterDetector::update()
{
    uint8_t s = palReadLine(HAL_GPIO_PIN_WATER_DETECTOR1);
    _state = s|(palReadLine(HAL_GPIO_PIN_WATER_DETECTOR2)<<1);
}

uint8_t CH_WaterDetector::read(void)
{
    return _state;
}
