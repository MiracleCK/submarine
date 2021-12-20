#pragma once
#include <stdint.h>
#include <AP_HAL/AP_HAL.h>

class CH_WaterDetector {

public:
    CH_WaterDetector(AP_HAL::AnalogSource *a1, AP_HAL::AnalogSource *a2);
    void init(void);
    void update(void);
    uint8_t read(void)
    {
        return state;
    }
    float get_analog1(void)
    {
        return ain1;
    }
    float get_analog2(void)
    {
        return ain2;
    }

private:
    uint8_t state;
    AP_HAL::AnalogSource *analog1, *analog2;
    float ain1, ain2;
};
