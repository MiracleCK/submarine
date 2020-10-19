#pragma once

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_ChibiOS_Namespace.h"

#include "shell.h"

class ChibiOS::Shell : public AP_HAL::Shell {
public:
    void     register_commands(ShellCommand* commands) override;
    int      printf(const char *fmt, ...) override;

    // register factory callback
    void register_factory_cb(AP_HAL::HAL::Callbacks* cb) override;
};
