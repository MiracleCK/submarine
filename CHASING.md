
# From ArduSbu to CI

* enable shell support @e2bf745d406e7afbf0883f0aa38cebf079920504
* use hwdef to describe M2 hardware, first version @816a19b846d20d0e5507b4f8153260bdf5f9448d
* johnson fixed spi driver issue @2bb744387f261a5dd3beec0f628cc9a07c9f5e5e
* support a mock baro @f94a0b8f391c951c9a30e80aeea9f2c24e5a5fdb
    + can be configed at hwdef, `define HAL_BARO_ALLOW_INIT_NO_BARO TRUE`
* change `chibios_board.mk` to support shell @0cdf35ffae6f570ea3b992d86e681521d43c1c4a
    + but still cannot build with test
* use `AP_Param::set_default_by_name` to set changed params as default when startup
* use hwdef to describe rc_in channel map @ cb32fc526d8e8315e5a90c95456164a61360c2c7
* map pwm value to DShot value @323c329a6661593b5b0fa58a6c45b867eb344671
* simplify and align sensor axis to body @7802a3afbcfbcdc600d836bea6eaec3dda147d49
    + use `ROTATION_CUSTOM_MAG` and `ROTATION_CUSTOM_IMU`
    + new body need to implement a new custom rotation
* not use `calculate_orientation` when calibrate compass @ d28942d30056b90a49c3b9afec2f7ce3bf0f4d7a
* extend shell commands
    + chibios not support change shell command at runtime, only can add new commands when thread created
        - this is the reason `const ShellCommand *scp = scfg->sc_commands;`
    + add a `AP_HAL::Shell`, and then implement chibi shell in `AP_HAL_ChibiOS`
* use `SUB_FRAME_CUSTOM` as the frame type
* calibration
    + use auto detect orientation accel calibration instead of old one