
#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_ChibiOS.h"

#include "Shell.h"
#include <shell.h>

#include "hal.h"
#include "chprintf.h"

using namespace ChibiOS;

extern const AP_HAL::HAL & hal;
static AP_HAL::Shell::ShellCommand* _shell_commands = NULL;

#define MAX_SHELL_COMMAND_COUNT 16
#define SHELL_CMD_STUFF(N) \
static void SHELL_CMD_STUFF##N(BaseSequentialStream *chp, int argc, char *argv[]) \
{ \
    _shell_commands[N].sc_function(argc, argv); \
}
#define CHIBI_SHELL_COMMAND_SLOT(N) {NULL, SHELL_CMD_STUFF##N},

#define REPEAT_16_MACRO(macro) \
macro(0) \
macro(1) \
macro(2) \
macro(3) \
macro(4) \
macro(5) \
macro(6) \
macro(7) \
macro(8) \
macro(9) \
macro(10) \
macro(11) \
macro(12) \
macro(13) \
macro(14) \
macro(15)

REPEAT_16_MACRO(SHELL_CMD_STUFF)

static ShellCommand chibi_shell_commands[MAX_SHELL_COMMAND_COUNT] = {
        REPEAT_16_MACRO(CHIBI_SHELL_COMMAND_SLOT)
};

ShellConfig shell_cfg = {
  (BaseSequentialStream *)&HAL_STDOUT_SERIAL,
  chibi_shell_commands
};

AP_HAL::HAL::Callbacks* g_factory_cb = NULL;

void Shell::register_commands(AP_HAL::Shell::ShellCommand* commands)
{
    _shell_commands = commands;
    uint32_t n = 0;
    while (n < MAX_SHELL_COMMAND_COUNT - 1)
    {
        if (commands[n].sc_name == NULL)
            break;
        chibi_shell_commands[n].sc_name = commands[n].sc_name;
        n++;
    }
    chibi_shell_commands[n].sc_name = NULL;
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