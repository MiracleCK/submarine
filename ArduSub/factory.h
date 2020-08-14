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

#include "factory_proto.h"

class Factory {
public:
    Factory(void);
	void init(void);
	void update(void);
	uint8_t isFactoryMode(void) const { return _test_mode; }
	void setHisiTestResult(uint8_t *result, uint32_t len);
	
private:
	Factory_proto *_uart_up;
    Factory_proto *_uart_down;
    
    uint8_t _test_mode;
    
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
	
    void _motor_test(void);
    int _mpu6000_test(void);
    int _ramtron_test(void);
    int _mmcsd_test(void);
    int _baro_test(void);
    int _battery_test(void);
    int _compass_test(void);
    int _gps_test(void);
    void _uart_update(void);
};


