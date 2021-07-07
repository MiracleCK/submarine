
#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_ChibiOS.h"

#include "Shell.h"

#include "hal.h"
#include "chprintf.h"

using namespace ChibiOS;

extern const AP_HAL::HAL & hal;

static void cmd_param(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_version(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_cali(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_led(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_mode(BaseSequentialStream *chp, int argc, char *argv[]);
static void cmd_commands(const char* name, BaseSequentialStream *chp, int argc, char *argv[]);
static AP_HAL::Shell::ShellCommand* _shell_commands = NULL;

static ShellCommand chibi_shell_commands[] = {
  {"param", cmd_param},
  {"version", cmd_version},
  {"reset", cmd_reset},
  {"cali", cmd_cali},
  {"led", cmd_led},
  {"mode", cmd_mode},
  {NULL, NULL} // this is the end of commands
};

ShellConfig shell_cfg = {
  (BaseSequentialStream *)&HAL_STDOUT_SERIAL,
  chibi_shell_commands
};

AP_HAL::HAL::Callbacks* g_factory_cb = NULL;

void Shell::register_commands(AP_HAL::Shell::ShellCommand* commands)
{
    _shell_commands = commands;
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

void cmd_param(BaseSequentialStream *chp, int argc, char *argv[]) {
    cmd_commands("param", chp, argc, argv);
}

void cmd_version(BaseSequentialStream *chp, int argc, char *argv[]) {
    cmd_commands("version", chp, argc, argv);
}

void cmd_reset(BaseSequentialStream *chp, int argc, char *argv[]) {
    cmd_commands("reset", chp, argc, argv);
}

void cmd_cali(BaseSequentialStream *chp, int argc, char *argv[]) {
    cmd_commands("cali", chp, argc, argv);
}

void cmd_led(BaseSequentialStream *chp, int argc, char *argv[]) {
    cmd_commands("led", chp, argc, argv);
}

void cmd_mode(BaseSequentialStream *chp, int argc, char *argv[]) {
    cmd_commands("mode", chp, argc, argv);
}

void cmd_commands(const char* name, BaseSequentialStream *chp, int argc, char *argv[]) {
    AP_HAL::Shell::ShellCommand* scp = _shell_commands;
    if (scp == NULL) {
        chprintf(chp, "commands is empty\r\n");
        return;
    }

    while (scp->sc_name != NULL) {
        if (strcmp(scp->sc_name, name) == 0) {
            scp->sc_function(argc, argv);
            return;
        }

        scp++;
    }
}