#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "Sub.h"

#include "factory.h"

#define FACTORY_TEST_IMU			(1)
#define FACTORY_TEST_COMPASS		(1)
#define FACTORY_TEST_BARO			(1)
#define FACTORY_TEST_RAMTRON		(1)
#define FACTORY_TEST_MMCSD			(0)
#define FACTORY_TEST_BATTERY		(1)
#define FACTORY_TEST_GPS			(0)

#define FACTORY_TEST_REUSLT_SEND_INTERVAL   (1000)
#define FACTORY_MOTOR_MAX_NUMBER            (4)

#define FACTORY_MPU6000_RESULT_BIT   0
#define FACTORY_BARO_RESULT_BIT      1
#define FACTORY_COMPASS_RESULT_BIT   2
#define FACTORY_RAMTRON_RESULT_BIT   3
#define FACTORY_MMCSD_RESULT_BIT     4
#define FACTORY_BATTERY_RESULT_BIT   5
#define FACTORY_GPS_RESULT_BIT       6
#define FACTORY_HISI_RESULT_BIT      7

const AP_Param::GroupInfo Factory::var_info[] = {
    // @Param: AGING_EN
    // @DisplayName: aging enable or disable
    // @Description: Used to enter or exit aging mode.
    // @Values: 1:enable,0:disable
    // @User: Advanced
    AP_GROUPINFO("AGING_EN",  0, Factory, _aging_enable, 0),

    // @Param: AGING_RES
    // @DisplayName: aging result
    // @Description: Used to return aging result.
    // @Values: bit[0-7]:imu,baro,compass,ramtron,mmcsd,batt,gps,hisi; 0:OK,1:err
    // @User: Advanced
    AP_GROUPINFO("AGING_RES", 1, Factory, _aging_result, 255),


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
        if (_test_mode || _mpu6000_result==0)
        {
            _mpu6000_result = _mpu6000_test();
        }

        /* EEPROM */
        if (_test_mode || _ramtron_result==0)
        {
            _ramtron_result = _ramtron_test();
        }

        /* SD card */
        if (_test_mode || _mmcsd_result==0)
        {
            _mmcsd_result = _mmcsd_test();
        }

        /* sensor presure */
        if (_test_mode || _baro_result==0)
        {
            _baro_result = _baro_test();
        }

        /* battery*/
        if (_test_mode || _batt_result==0)
        {
            _batt_result = _battery_test();
        }

        /* sensor compass */
        if (_test_mode || _compass_result==0)
        {
            _compass_result = _compass_test();
        }

        /* sensor gps */
        if (_test_mode || _gps_result==0)
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

		if(timesec>10)
			tested = 1;
			
		if(timesec%60==0 && 
		   _aging_result!=result) {
			_aging_result.set_and_save(result);
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
		printf("depth %f\r\n", depth);
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
    _hisi_result_new(0)
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
	AP_AHRS &ahrs = AP::ahrs();

    ahrs.get_position(current_loc);
	depth = current_loc.alt * 0.01f;	
	
    sub.set_mode(ALT_HOLD, ModeReason::RC_COMMAND);
	sub.motors.armed(TRUE);

	if(depth>-0.5) 
		sub.channel_throttle->set_radio_in(1350);
	else if(depth<-0.6)
		sub.channel_throttle->set_radio_in(1650);
	else 
		sub.channel_throttle->set_radio_in(1500);
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

	if (ins.get_accel_health_all() && ins.get_gyro_health_all()) {
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

	if (baro.healthy()) {
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

    if (battery.num_instances() > 0 && battery.healthy()) {
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

    if (compass.enabled() && compass.healthy()) {
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

