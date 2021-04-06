#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_Math/AP_Math.h>

#include "AP_Compass.h"
#include "AP_Compass_Backend.h"


#ifndef HAL_COMPASS_VCM1193_I2C_ADDR 
 #define HAL_COMPASS_VCM1193_I2C_ADDR     0x0C
#endif

class AP_Compass_VCM1193 : public AP_Compass_Backend
{
public:
    static AP_Compass_Backend *probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
    								 bool force_external,
                                     enum Rotation rotation);

    static constexpr const char *name = "VCM1193";

    void read() override;

    ~AP_Compass_VCM1193() { }

private:
    AP_Compass_VCM1193(AP_HAL::OwnPtr<AP_HAL::Device> dev);

    bool init(enum Rotation rotation, bool force_external);

    bool _read_sample();

    bool _hardware_init();
    void _update();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    int16_t _mag_x;
    int16_t _mag_y;
    int16_t _mag_z;

    uint8_t _compass_instance;
    bool _initialised;
};
