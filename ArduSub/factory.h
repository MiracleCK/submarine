#pragma once
/*
  This is the factory Sub class
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////

#include <cmath>
#include <stdio.h>
#include <stdarg.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#include "factory_proto.h"

class Factory : public AP_HAL::HAL::Callbacks {
public:
    Factory(void);
    void test_check(void);
    void aging_check(void);
	void setup(void) override;
	void loop(void) override;
	uint8_t isFactoryTestMode(void) const { return _test_mode; }
	uint8_t isFactoryAgingMode(void) const { return _aging_mode; }
	void setHisiTestResult(uint8_t *result, uint32_t len);

	static const struct AP_Param::GroupInfo var_info[];	
private:
	
	Factory_proto *_uart_up;
    Factory_proto *_uart_down;
    
    uint8_t _test_mode;
    uint8_t _aging_mode;
    AP_Int8 _aging_enable;
    AP_Int16 _aging_time;
    AP_Int16 _aging_result[2];

    struct Location _current_loc;
  	float _depth; 
    
	uint32_t _motor_time;
	uint8_t _motor_state;

    uint32_t _mpu6000_result;
    uint32_t _ramtron_result;
    uint32_t _mmcsd_result;
    uint32_t _baro_result;
    uint32_t _compass_result;
    uint32_t _usb_result;
    uint32_t _batt_result;
    uint32_t _gps_result;

    uint16_t _hisi_result;
    uint16_t _hisi_result_new;

    uint32_t _result_timestamp;

	Vector3f _imu_gyro[2];
    Vector3f _imu_accel[2];
    Vector3f _imu_mag[2];
    float _baro_press[2];
	float _baro_temp[2];
	float _batt_voltage;
	float _batt_current;
	uint8_t _batt_remaining;
	
    void _motor_test(void);
    void _aging_test(void);
    int _mpu6000_test(void);
    int _storage_test(void);
    int _ramtron_test(void);
    int _mmcsd_test(void);
    int _baro_test(void);
    int _battery_test(void);
    int _compass_test(void);
    int _gps_test(void);
    void _uart_update(void);
};

extern Factory factory;

