
#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_ChibiOS.h"

#include "Shell.h"
#include <shell.h>

#include "hal.h"
#include "chprintf.h"

using namespace ChibiOS;

extern const AP_HAL::HAL & hal;

ShellConfig shell_cfg = {
  (BaseSequentialStream *)&HAL_STDOUT_SERIAL,
  NULL
};

AP_HAL::HAL::Callbacks* g_factory_cb = NULL;

void Shell::register_commands(AP_HAL::Shell::ShellCommand* commands)
{
    shell_cfg.hc_commands = (HalCommand *)commands;
}

void Shell::register_factory_cb(AP_HAL::HAL::Callbacks* cb)
{
    g_factory_cb = cb;
}

int Shell::printf(const char *fmt, ...)
{
    va_list ap;
    int formatted_bytes;

    va_start(ap, fmt);
    formatted_bytes = chvprintf(shell_cfg.sc_channel, fmt, ap);
    va_end(ap);

    return formatted_bytes;
}