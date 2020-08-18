#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include "Sub.h"

#include "factory.h"

#define FACTORY_TEST_REUSLT_SEND_INTERVAL   (1000)
#define FACTORY_MOTOR_MAX_NUMBER            (4)

#define FACTORY_MPU6000_RESULT_BIT   0

#define FACTORY_BARO_RESULT_BIT      1
#define FACTORY_GPS_RESULT_BIT       1

#define FACTORY_COMPASS_RESULT_BIT   2
#define FACTORY_RAMTRON_RESULT_BIT   3
#define FACTORY_MMCSD_RESULT_BIT     4
#define FACTORY_BATTERY_RESULT_BIT   5
#define FACTORY_HISI_RESULT_BIT      6

void Factory::check()
{
	uint8_t test_pin;
	
	printf("factory check\r\n");
	
	test_pin = palReadLine(HAL_GPIO_PIN_TEST);
	if(!test_pin)
		hal.shell->register_factory_cb(this);
}

void Factory::setup()
{
    printf("enter factory test mode\r\n");

    _test_mode = 1;
    _motor_time = AP_HAL::millis();
    _result_timestamp = AP_HAL::millis();
    sub.motors.armed(TRUE);

    _uart_up->init();
	_uart_down->init();
}

void Factory::loop()
{	
	uint8_t result = 0;
    //static uint32_t tested = 0;
    static uint32_t timesec = 0;
    
	if(!_test_mode)
		return ;

	_uart_update();

	_motor_test();

	//if (!tested)
    //{
        /* sensor mpu6000 */
        //if (0 != _mpu6000_result)
        {
            _mpu6000_result = _mpu6000_test();
        }

        /* EEPROM */
        //if (0 != _ramtron_result)
        {
            _ramtron_result = _ramtron_test();
        }

        /* SD card */
        //if (0 != _mmcsd_result)
        {
            _mmcsd_result = _mmcsd_test();
        }

        /* sensor presure */
        //if (0 != _baro_result)
        {
            _baro_result = _baro_test();
        }

        /* battery*/
        //if (0 != _batt_result)
        {
            _batt_result = _battery_test();
        }

        /* sensor compass */
        //if (0 != _compass_result)
        {
            _compass_result = _compass_test();
        }

        /* sensor gps */
        //if (0 != _gps_result)
        {
            _gps_result = _gps_test();
        }

        //tested = 1;
    //}

    result = 0;
    result = _mpu6000_result << FACTORY_MPU6000_RESULT_BIT 
           | _gps_result  << FACTORY_GPS_RESULT_BIT
           | _compass_result << FACTORY_COMPASS_RESULT_BIT
           | _ramtron_result << FACTORY_RAMTRON_RESULT_BIT
           | _mmcsd_result   << FACTORY_MMCSD_RESULT_BIT
           | _batt_result    << FACTORY_BATTERY_RESULT_BIT
           | (_hisi_result > 0)    << FACTORY_HISI_RESULT_BIT;

	if (AP_HAL::millis() - _result_timestamp >= FACTORY_TEST_REUSLT_SEND_INTERVAL)
	{
		if(_hisi_result_new)
		{
		    _uart_down->sendFactoryTestMsg(FACTORY_TEST_STM32_RESULT_MSGID, result, _hisi_result);
		    
		}
		else if (timesec > 35)
		{
		    _uart_down->sendFactoryTestMsg(FACTORY_TEST_STM32_RESULT_MSGID, result, _hisi_result);
		}

		_result_timestamp = AP_HAL::millis();
		timesec++;

#if 1
		printf("\r\n");
		printf("result 0x%x\r\n", result);
		printf("_mpu6000_result %d\r\n", _mpu6000_result);
		printf("_gps_result %d\r\n", _gps_result);
		printf("_compass_result %d\r\n", _compass_result);
		printf("_ramtron_result %d\r\n", _ramtron_result);
		printf("_mmcsd_result %d\r\n", _mmcsd_result);
		printf("_batt_result %d\r\n", _batt_result);
		printf("_hisi_result 0x%x\r\n", _hisi_result);
		printf("\r\n");
		result = result;
#endif
	}
}

Factory::Factory(void):
	_uart_up(&g_uart_up_port),
    _uart_down(&g_uart_down_port),
    _motor_state(0),
    _mpu6000_result(1),
    _compass_result(1),
    _baro_result(1),
    _ramtron_result(1),
    _mmcsd_result(1),
    _usb_result(1),
    _gps_result(1),
    _batt_result(1),
    _hisi_result(0x7f),
    _hisi_result_new(0)
{
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

void Factory::_motor_test()
{
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
	const AP_InertialSensor &ins = AP::ins();

	if (ins.get_accel_health_all() && ins.get_gyro_health_all()) {
        return 0;
    }

    return 1;
}

int Factory::_ramtron_test()
{
#if HAL_WITH_RAMTRON
	AP_HAL::Storage *st = hal.storage;

	if(st->healthy()) {
		return 0;
	}
	return 1;
#else
    return 0;
#endif
}

int Factory::_mmcsd_test()
{
    return 0;
}

int Factory::_baro_test()
{
    return 0;
}

int Factory::_battery_test()
{
	//const AP_BattMonitor &battery = AP::battery();

    //if (battery.num_instances() > 0 && battery.healthy()) {
    //    return 0;
    //}
    return 0;
}

int Factory::_compass_test()
{
	const Compass &compass = AP::compass();

    if (compass.enabled() && compass.healthy()) {
        return 0;
    }
    return 1;
}

int Factory::_gps_test()
{
    const AP_GPS &gps = AP::gps();

    if (gps.is_healthy() && gps.status() >= AP_GPS::NO_FIX) {
        return 0;
    }
    return 1;
}

Factory factory;

