#pragma once

#include <stdint.h>
#include "AP_HAL_Namespace.h"

class AP_HAL::Shell {
public:
    typedef void (*shellcmd_t)(int argc, char *argv[]);

    typedef struct {
        const char            *sc_name;          
        shellcmd_t            sc_function;       
    } ShellCommand;

    virtual void     register_commands(ShellCommand* commands) = 0;
    virtual int      printf(const char *fmt, ...) = 0;

    virtual void register_factory_cb(AP_HAL::HAL::Callbacks* cb) = 0;
};
