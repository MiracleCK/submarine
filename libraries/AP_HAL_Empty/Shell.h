#pragma once

#include "AP_HAL_Empty.h"

class Empty::Shell : public AP_HAL::Shell {
public:
    void     register_commands(ShellCommand* commands) override {};
    int      printf(const char *fmt, ...) override {return 0;};
};
