#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "Sub.h"

#include "factory.h"


#define FACTORY_TEST_IMU			(1)
#define FACTORY_TEST_COMPASS		(1)
#define FACTORY_TEST_BARO			(0)
#define FACTORY_TEST_RAMTRON		(1)
#define FACTORY_TEST_MMCSD			(0)
#define FACTORY_TEST_BATTERY		(0)
#define FACTORY_TEST_GPS			(1)

#define FACTORY_TEST_REUSLT_SEND_INTERVAL   (1000)
#define IMU_GYRO_ERROR_RANGE		10.0f
#define IMU_ACCEL_ERROR_RANGE		10.0f
#define IMU_COMPASS_ERROR_RANGE		10.0f
#define BARO_PRESS_ERROR_RANGE		2.0f
#define BARO_TEMP_ERROR_RANGE		2.0f
#define BATT_VOL_MIN				18000.0f
#define BATT_VOL_MAX				25200.0f

#define FACTORY_MPU6000_RESULT_BIT   0
#define FACTORY_BARO_RESULT_BIT      1
#define FACTORY_COMPASS_RESULT_BIT   2
#define FACTORY_RAMTRON_RESULT_BIT   3
#define FACTORY_MMCSD_RESULT_BIT     4
#define FACTORY_BATTERY_RESULT_BIT   5
#define FACTORY_GPS_RESULT_BIT       6
#define FACTORY_HISI_RESULT_BIT      7

#define diff(a, b) ((a) > (b) ? ((a)-(b)) : ((b)-(a)))

const AP_Param::GroupInfo Factory::var_info[] = {
    // @Param: AGING_ENABLE
    // @DisplayName: aging enable or disable
    // @Description: Used to enter or exit aging mode.
    // @Values: 1:enable,0:disable
    // @User: Advanced
    AP_GROUPINFO("AGING_ENABLE",  0, Factory, _aging_enable, 0),

    // @Param: AGING_TIME
    // @DisplayName: submarine aging time
    // @Description: Used to return aging time.
    // @Values: aging time, in 1min units
    // @User: Advanced
    AP_GROUPINFO("AGING_TIME", 1, Factory, _aging_time, 0),

    // @Param: AGING_RESULT
    // @DisplayName: submarine aging result
    // @Description: Used to return aging result.
    // @Values: bit[0-7]:imu,baro,compass,ramtron,mmcsd,batt,gps,hisi; 0:OK,1:err
    // @User: Advanced
    AP_GROUPINFO("AGING_RESULT", 2, Factory, _aging_result[0], 255),

	// @Param: AGING_RESULT2
    // @DisplayName: hisi aging result
    // @Description: Used to return aging result.
    // @Values: hisi define; 0:OK,1:err
    // @User: Advanced
    AP_GROUPINFO("AGING_RESULT2", 3, Factory, _aging_result[1], 255),
    
    AP_GROUPEND
};

void Factory::test_check()
{
	uint8_t test_pin;
	
	test_pin = palReadLine(HAL_GPIO_PIN_TEST);
	if(!test_pin)
		_test_mode = 1;

	if(_test_mode) {
		printf("enter factory test mode\r\n");
		hal.shell->register_factory_cb(this);
	}
}

void Factory::aging_check()
{
	if(_aging_enable) {
		_aging_enable.set_and_save(0);
		_aging_mode = 1;
	}
	
	if(_aging_mode) {
		printf("enter factory aging mode\r\n");
		hal.shell->register_factory_cb(this);
	}
}

void Factory::setup()
{
    _motor_time = AP_HAL::millis();
    _result_timestamp = AP_HAL::millis();

    _uart_up->init();
	_uart_down->init();

	if(_aging_mode) {
		_aging_time.set_and_save(0);
		_aging_result[0].set_and_save(255);
		_aging_result[1].set_and_save(255);

		//sub.channel_roll->set_radio_in(1500);
		//sub.channel_pitch->set_radio_in(1500);
		//sub.channel_throttle->set_radio_in(1500);
		//sub.channel_yaw->set_radio_in(1500);
		//sub.channel_forward->set_radio_in(1500);
		//sub.channel_lateral->set_radio_in(1500);
		//AP_Param::set_by_name("RC_OPTIONS", 3);
	}
}

void Factory::loop()
{	
	uint8_t result = 0xff;
    static uint32_t tested = 0;
    static uint32_t timesec = 0;

	_uart_update();
	
	if(_test_mode) {
		_motor_test();
	} else {
		_aging_test();
	}
	
	if (tested)
    {
        /* sensor mpu6000 */
        //if (_test_mode || _mpu6000_result==0)
        {
            _mpu6000_result = _mpu6000_test();
        }

        /* EEPROM */
        //if (_test_mode || _ramtron_result==0)
        {
            _ramtron_result = _ramtron_test();
        }

        /* SD card */
        //if (_test_mode || _mmcsd_result==0)
        {
            _mmcsd_result = _mmcsd_test();
        }

        /* sensor presure */
        //if (_test_mode || _baro_result==0)
        {
            _baro_result = _baro_test();
        }

        /* battery*/
        //if (_test_mode || _batt_result==0)
        {
            _batt_result = _battery_test();
        }

        /* sensor compass */
        //if (_test_mode || _compass_result==0)
        {
            _compass_result = _compass_test();
        }

        /* sensor gps */
        //if (_test_mode || _gps_result==0)
        {
            _gps_result = _gps_test();
        }

        result = 0;
	    result = _mpu6000_result << FACTORY_MPU6000_RESULT_BIT 
	           | _baro_result  << FACTORY_BARO_RESULT_BIT
	           | _compass_result << FACTORY_COMPASS_RESULT_BIT
	           | _ramtron_result << FACTORY_RAMTRON_RESULT_BIT
	           | _mmcsd_result   << FACTORY_MMCSD_RESULT_BIT
	           | _batt_result    << FACTORY_BATTERY_RESULT_BIT
	           | _gps_result  << FACTORY_GPS_RESULT_BIT
	           | (_hisi_result > 0)    << FACTORY_HISI_RESULT_BIT;
    }
    
	if (AP_HAL::millis() - _result_timestamp >= FACTORY_TEST_REUSLT_SEND_INTERVAL)
	{
		_result_timestamp = AP_HAL::millis();
		timesec++;
		
		_uart_down->sendFactoryTestMsg(FACTORY_TEST_STM32_RESULT_MSGID, result, _hisi_result);

		if(timesec>10) {
			tested = 1;
		}
			
		if(_aging_mode && timesec%60==0) {
			_aging_result[0].set_and_save_ifchanged(result);
			_aging_result[1].set_and_save_ifchanged(_hisi_result);
			_aging_time.set_and_save(++_time_min);
		}

#if 1
		printf("\r\n");
		printf("result 0x%x\r\n", result);
		printf("_mpu6000_result %d\r\n", _mpu6000_result);
		printf("_baro_result %d\r\n", _baro_result);
		printf("_compass_result %d\r\n", _compass_result);
		printf("_ramtron_result %d\r\n", _ramtron_result);
		printf("_mmcsd_result %d\r\n", _mmcsd_result);
		printf("_batt_result %d\r\n", _batt_result);
		printf("_gps_result %d\r\n", _gps_result);
		printf("_hisi_result 0x%x\r\n", _hisi_result);
		printf("\r\n");

		printf("_imu_gyro.x %f %f %f\r\n", _imu_gyro[0].x, _imu_gyro[1].x, diff(_imu_gyro[0].x, _imu_gyro[1].x));
		printf("_imu_gyro.y %f %f %f\r\n", _imu_gyro[0].y, _imu_gyro[1].y, diff(_imu_gyro[0].y, _imu_gyro[1].y));
		printf("_imu_gyro.z %f %f %f\r\n", _imu_gyro[0].z, _imu_gyro[1].z, diff(_imu_gyro[0].z, _imu_gyro[1].z));
		
		printf("_imu_accel.x %f %f %f\r\n", _imu_accel[0].x, _imu_accel[1].x, diff(_imu_accel[0].x, _imu_accel[1].x));
		printf("_imu_accel.y %f %f %f\r\n", _imu_accel[0].y, _imu_accel[1].y, diff(_imu_accel[0].y, _imu_accel[1].y));
		printf("_imu_accel.z %f %f %f\r\n", _imu_accel[0].z, _imu_accel[1].z, diff(_imu_accel[0].z, _imu_accel[1].z));
		
		printf("_imu_mag.x %f %f %f\r\n", _imu_mag[0].x, _imu_mag[1].x, diff(_imu_mag[0].x, _imu_mag[1].x));
		printf("_imu_mag.y %f %f %f\r\n", _imu_mag[0].y, _imu_mag[1].y, diff(_imu_mag[0].y, _imu_mag[1].y));
		printf("_imu_mag.z %f %f %f\r\n", _imu_mag[0].z, _imu_mag[1].z, diff(_imu_mag[0].z, _imu_mag[1].z));
		printf("\r\n");
#endif
	}
}

Factory::Factory(void):
	_uart_up(&g_uart_up_port),
    _uart_down(&g_uart_down_port),
    _test_mode(0),
    _motor_state(0),
    _mpu6000_result(0),
    _compass_result(0),
    _baro_result(0),
    _ramtron_result(0),
    _mmcsd_result(0),
    _usb_result(0),
    _gps_result(0),
    _batt_result(0),
    _hisi_result(0x7f),
    _hisi_result_new(0),
    _time_min(0)
{
	AP_Param::setup_object_defaults(this, var_info);
}

void Factory::setHisiTestResult(uint8_t *result, uint32_t len)
{
	uint32_t i;

	_hisi_result = 0;

	//printf("data len =%d \n",len);
	for (i = 0; i < len; i++)
	{
		_hisi_result |= result[i] << i;
	}
	//printf("_hisi_result =%x \n", _hisi_result);
	_hisi_result_new = 1;
	return;
}

void Factory::_uart_update()
{
	_uart_up->read();
	_uart_down->read();
}

void Factory::_aging_test()
{
    sub.set_mode(STABILIZE, ModeReason::RC_COMMAND);
	sub.motors.armed(TRUE);
}

void Factory::_motor_test()
{
	sub.motors.armed(TRUE);
	sub.motors.set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    if (AP_HAL::millis() - _motor_time >= 1000)
    {
        //printf("motor test...\r\n");
        switch(_motor_state)
        {
            case 0:
                sub.motors.set_lateral(0.5);
                _motor_state = 1;
                break;
            case 1:
                sub.motors.set_lateral(0);
                _motor_state = 2;
                break;
            case 2:
                sub.motors.set_lateral(-0.5);
                _motor_state = 3;
                break; 
            case 3:
                sub.motors.set_lateral(0);
                _motor_state = 0;
                break;
            default:break;
        }

        _motor_time = AP_HAL::millis();
    }
}

int Factory::_mpu6000_test()
{
#if FACTORY_TEST_IMU
	const AP_InertialSensor &ins = AP::ins();
	static uint8_t init = 0;
	static uint8_t i = 0;

	if (ins.get_accel_health_all() && ins.get_gyro_health_all()) {
		if(init==0) {
			_imu_gyro[0] = _imu_gyro[1] = ins.get_gyro() * 1000.0f;
			_imu_accel[0] = _imu_accel[1] = ins.get_accel() * 1000.0f / GRAVITY_MSS;
			init = 1;
		} else {
			_imu_gyro[i] = ins.get_gyro() * 1000.0f;
			_imu_accel[i] = ins.get_accel() * 1000.0f / GRAVITY_MSS;
			i = (i+1)%2;
		}

		//if(_aging_mode)
		//	return 0;

		if(diff(_imu_gyro[0].x, _imu_gyro[1].x) < IMU_GYRO_ERROR_RANGE &&
		   diff(_imu_gyro[0].y, _imu_gyro[1].y) < IMU_GYRO_ERROR_RANGE &&
		   diff(_imu_gyro[0].z, _imu_gyro[1].z) < IMU_GYRO_ERROR_RANGE &&
		   diff(_imu_accel[0].x, _imu_accel[1].x) < IMU_ACCEL_ERROR_RANGE &&
		   diff(_imu_accel[0].y, _imu_accel[1].y) < IMU_ACCEL_ERROR_RANGE &&
		   diff(_imu_accel[0].z, _imu_accel[1].z) < IMU_ACCEL_ERROR_RANGE)
        	return 0;
    }

    return 1;
#else
	return 0;
#endif
}

int Factory::_storage_test()
{
	AP_HAL::Storage *st = hal.storage;

	if(st->healthy()) {
		if (sub.g.format_version.load() &&
	        sub.g.format_version == Parameters::k_format_version)
			return 0;
	}
	return 1;
}

int Factory::_ramtron_test()
{
#if FACTORY_TEST_RAMTRON && HAL_WITH_RAMTRON
	return _storage_test();
#else
    return 0;
#endif
}

int Factory::_mmcsd_test()
{
#if FACTORY_TEST_MMCSD && defined(USE_POSIX)
	return _storage_test();
#else
	return 0;
#endif
}

int Factory::_baro_test()
{
#if FACTORY_TEST_BARO
	const AP_Baro &baro = AP::baro();
	static uint8_t init = 0;
	static uint8_t i = 0;
	
	if (baro.healthy()) {
        if(init==0) {
			_baro_press[0] = _baro_press[1] = baro.get_pressure() * 0.01f;
			_baro_temp[0] = _baro_temp[1] = baro.get_temperature()*100;
			init = 1;
		} else {
			_baro_press[i] = baro.get_pressure() * 0.01f;
			_baro_temp[i] = baro.get_temperature()*100;
			i = (i+1)%2;
		}

		//if(_aging_mode)
		//	return 0;
			
		if(diff(_baro_press[0], _baro_press[1]) < BARO_PRESS_ERROR_RANGE &&
		   diff(_baro_temp[0], _baro_temp[1]) < BARO_TEMP_ERROR_RANGE)
        	return 0;
    }
    return 1;
#else
	return 0;
#endif
}

int Factory::_battery_test()
{
#if FACTORY_TEST_BATTERY
    const AP_BattMonitor &battery = AP::battery();

    if (battery.num_instances() && 
        battery.healthy() && 
        battery.current_amps(_batt_current)) {
        _batt_remaining = battery.capacity_remaining_pct(); // in %
        _batt_current *= 100; //in 10mA units
        _batt_voltage = battery.voltage() * 1000; //mv
        if(_batt_voltage > BATT_VOL_MIN &&
           _batt_voltage < BATT_VOL_MAX)
        	return 0;
    }

    return 1;
#else
	return 0;
#endif
}

int Factory::_compass_test()
{
#if FACTORY_TEST_COMPASS
	const Compass &compass = AP::compass();
	static uint8_t init = 0;
	static uint8_t i = 0;
	
    if (compass.enabled() && compass.healthy()) {
    	if(init==0) {
			_imu_mag[0] = _imu_mag[1] = compass.get_field();
			init = 1;
		} else {
			_imu_mag[i] = compass.get_field();
			i = (i+1)%2;
		}

		//if(_aging_mode)
		//	return 0;
			
		if(diff(_imu_mag[0].x, _imu_mag[1].x) < IMU_COMPASS_ERROR_RANGE &&
		   diff(_imu_mag[0].y, _imu_mag[1].y) < IMU_COMPASS_ERROR_RANGE &&
		   diff(_imu_mag[0].z, _imu_mag[1].z) < IMU_COMPASS_ERROR_RANGE)
        	return 0;
    }
    return 1;
#else
	return 0;
#endif
}

int Factory::_gps_test()
{
#if FACTORY_TEST_GPS
    const AP_GPS &gps = AP::gps();

    if (gps.is_healthy() && gps.status() >= AP_GPS::NO_FIX) {
        return 0;
    }
    return 1;
#else
	return 0;
#endif
}

Factory factory;

