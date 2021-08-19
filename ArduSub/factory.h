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

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL

#include "factory_proto.h"

class Factory : public AP_HAL::HAL::Callbacks {
public:
    Factory(void);
    void test_check(void);
    void mode_check(void);
	void setup(void) override;
	void loop(void) override;
	uint8_t isFactoryTestMode(void) const { return _test_mode; }
	uint8_t isFactoryAgingMode(void) const { return _aging_mode; }
	void setHisiTestResult(uint8_t *result, uint32_t len);

	static const struct AP_Param::GroupInfo var_info[];	
private:
	
	Factory_proto *_uart_up;
    Factory_proto *_uart_down;

    AP_Int8 _aging_enable;
    AP_Int16 _aging_time;
    AP_Int16 _aging_result[2];

    AP_Float _aging_gyro[3];
    AP_Float _aging_gyro_vari[3];
    AP_Float _aging_accel[3];
    AP_Float _aging_accel_vari[3];
    AP_Float _aging_mag[3];
    AP_Float _aging_mag_vari[3];
    AP_Float _aging_baro[2];
    AP_Float _aging_baro_vari[2];

    AP_Int8 _washing_enable;
    
    uint8_t _test_mode;
    uint8_t _aging_mode;
    uint8_t _washing_mode;
    uint16_t _time_min;

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
	uint64_t _imu_gyro_n;
	Vector3f _imu_gyro_sum;
	Vector3f _imu_gyro_sum2;
	Vector3f _imu_gyro_aver;
	Vector3f _imu_gyro_aver2;
	Vector3f _imu_gyro_vari;
	
    Vector3f _imu_accel[2];
    uint64_t _imu_accel_n;
	Vector3f _imu_accel_sum;
	Vector3f _imu_accel_sum2;
	Vector3f _imu_accel_aver;
	Vector3f _imu_accel_aver2;
	Vector3f _imu_accel_vari;
	
    Vector3f _imu_mag[2];
    uint64_t _imu_mag_n;
	Vector3f _imu_mag_sum;
	Vector3f _imu_mag_sum2;
	Vector3f _imu_mag_aver;
	Vector3f _imu_mag_aver2;
	Vector3f _imu_mag_vari;
	
    float _baro_press[2];
    uint64_t _baro_press_n;
	float _baro_press_sum;
	float _baro_press_sum2;
	float _baro_press_aver;
	float _baro_press_aver2;
	float _baro_press_vari;
	
	float _baro_temp[2];
	uint64_t _baro_temp_n;
	float _baro_temp_sum;
	float _baro_temp_sum2;
	float _baro_temp_aver;
	float _baro_temp_aver2;
	float _baro_temp_vari;
	
	float _batt_voltage;
	float _batt_current;
	uint8_t _batt_remaining;
	
    void _motor_test(void);
    void _aging_test(void);
    void _washing_test(void);
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

#endif
