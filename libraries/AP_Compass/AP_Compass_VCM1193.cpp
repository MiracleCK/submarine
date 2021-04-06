/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

#include "AP_Compass_VCM1193.h"

extern const AP_HAL::HAL &hal;

// Registers
#define VCM1193_MAG_REG_HXL          0x00
#define VCM1193_MAG_REG_HXH          0x01
#define VCM1193_MAG_REG_HYL          0x02
#define VCM1193_MAG_REG_HYH          0x03
#define VCM1193_MAG_REG_HZL          0x04
#define VCM1193_MAG_REG_HZH          0x05
#define VCM1193_MAG_REG_CTRL_REG2    0x0A
#define VCM1193_MAG_REG_CTRL_REG1    0x0B
#define VCM1193_MAG_REG_WHO_AM_I     0x0C

AP_Compass_VCM1193::AP_Compass_VCM1193(AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : _dev(std::move(dev))
{
}

AP_Compass_Backend *AP_Compass_VCM1193::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
											  bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_VCM1193 *sensor = new AP_Compass_VCM1193(std::move(dev));
    if (!sensor || !sensor->init(rotation, force_external)) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


bool AP_Compass_VCM1193::init(enum Rotation rotation, bool force_external)
{

    bool success = _hardware_init();

    if (!success) {
        return false;
    }

    _initialised = true;

    // perform an initial read
    read();
    
    /* register the compass instance in the frontend */
    _compass_instance = register_compass();

    set_rotation(_compass_instance, rotation);

    _dev->set_device_type(DEVTYPE_VCM1193);
    set_dev_id(_compass_instance, _dev->get_bus_id());

    if (force_external) {
        set_external(_compass_instance, true);
    }

    // read at 75Hz
    _dev->register_periodic_callback(13333, FUNCTOR_BIND_MEMBER(&AP_Compass_VCM1193::_update, void)); 

    return true;
}

bool AP_Compass_VCM1193::_hardware_init()
{

    AP_HAL::Semaphore *bus_sem = _dev->get_semaphore();
    if (!bus_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        AP_HAL::panic("VCM1193: Unable to get semaphore");
    }

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    bool ret=false;
    
    _dev->set_retries(5);
    
    uint8_t sig = 0;
    bool ack = _dev->read_registers(VCM1193_MAG_REG_WHO_AM_I, &sig, 1);    
    if (!ack || sig != 0x82) goto exit;

    ack = _dev->write_register(VCM1193_MAG_REG_CTRL_REG1, 0x00); //  soft reset
    if (!ack) goto exit;

    //hal.scheduler->delay(20);
    
    ack = _dev->write_register(VCM1193_MAG_REG_CTRL_REG2, 0x45); // active mode 100 Hz ODR
    if (!ack) goto exit;

    ret = true;

    _dev->set_retries(3);

    hal.scheduler->delay(20);
    
    printf("VCM1193 found on bus 0x%x, chip id 0x%x\r\n", (uint16_t)_dev->get_bus_id(), sig);

exit:
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    bus_sem->give();
    return ret;
}


// Read Sensor data
bool AP_Compass_VCM1193::_read_sample()
{
    uint8_t buf[6];
    bool ack = _dev->read_registers(VCM1193_MAG_REG_HXL, buf, 6);
    if (!ack) {
        return false;
    }

    _mag_x = -(int16_t)(buf[1] << 8 | buf[0]);
    _mag_y = -(int16_t)(buf[3] << 8 | buf[2]);
    _mag_z = -(int16_t)(buf[5] << 8 | buf[4]);

    return true;
}


#define MAG_SCALE (1.0f/10000 / 0.0001f * 1000)  // 1 Tesla full scale of +-10000, 1 Gauss = 0,0001 Tesla, library needs milliGauss

void AP_Compass_VCM1193::_update()
{
    if (!_read_sample()) {
        return;
    }

#if 0
    printf("mag [%d %d %d]\r\n", _mag_x, _mag_y, _mag_z);
#endif

	const float range_scale = 1000.0f / 3000.0f;
    Vector3f raw_field = Vector3f((float)_mag_x * range_scale, (float)_mag_y * range_scale, (float)_mag_z * range_scale);

    accumulate_sample(raw_field, _compass_instance);
}


// Read Sensor data
void AP_Compass_VCM1193::read()
{
    if (!_initialised) {
        return;
    }

    drain_accumulated_samples(_compass_instance);
}
