
#include <string.h>

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_FWVersion.h>

#include <AP_Common/Location.h>
#include "Sub.h"

Location target_loc;
bool pos_get_flag = false;
bool pos_set_flag = false;

extern const AP_HAL::HAL& hal;

extern void cmd_cali(int argc, char *argv[]);

typedef struct {
    const char* param_short_name;
    const char* param_name;
}param_name_t;

const param_name_t params[] = {
    {"rp", "ATC_RAT_RLL_P"},
    {"ri", "ATC_RAT_RLL_I"},
    {"rd", "ATC_RAT_RLL_D"},
    {"pp", "ATC_RAT_PIT_P"},
    {"pi", "ATC_RAT_PIT_I"},
    {"pd", "ATC_RAT_PIT_D"},
    {"yp", "ATC_RAT_YAW_P"},
    {"yi", "ATC_RAT_YAW_I"},
    {"yd", "ATC_RAT_YAW_D"},
    {"pzp", "PSC_POSZ_P"},
    {"vzp", "PSC_VELZ_P"},
    {"azp", "PSC_ACCZ_P"},
    {"azi", "PSC_ACCZ_I"},
    {"azd", "PSC_ACCZ_D"},
    {"arp", "ATC_ANG_RLL_P"},
    {"app", "ATC_ANG_PIT_P"},
    {"ayp", "ATC_ANG_YAW_P"},
    {"arfe", "ATC_RATE_FF_ENAB"},
    {"m1m", "MOT_1_MAPPING"},
    {"m2m", "MOT_2_MAPPING"},
    {"m3m", "MOT_3_MAPPING"},
    {"m4m", "MOT_4_MAPPING"},
    {"m5m", "MOT_5_MAPPING"},
    {"m6m", "MOT_6_MAPPING"},
    {"m7m", "MOT_7_MAPPING"},
    {"m8m", "MOT_8_MAPPING"},
    {"m1d", "MOT_1_DIRECTION"},
    {"m2d", "MOT_2_DIRECTION"},
    {"m3d", "MOT_3_DIRECTION"},
    {"m4d", "MOT_4_DIRECTION"},
    {"m5d", "MOT_5_DIRECTION"},
    {"m6d", "MOT_6_DIRECTION"},
    {"m7d", "MOT_7_DIRECTION"},
    {"m8d", "MOT_8_DIRECTION"},
    {"m1z", "MOT_1_DEADZONE"},
	{"m2z", "MOT_2_DEADZONE"},
	{"m3z", "MOT_3_DEADZONE"},
	{"m4z", "MOT_4_DEADZONE"},
	{"m5z", "MOT_5_DEADZONE"},
	{"m6z", "MOT_6_DEADZONE"},
	{"m7z", "MOT_7_DEADZONE"},
	{"m8z", "MOT_8_DEADZONE"},
    {"amax", "ANGLE_MAX"},
    {"atx", "AHRS_TRIM_X"},
    {"aty", "AHRS_TRIM_Y"},
    {"atz", "AHRS_TRIM_Z"},
    {"gap", "GND_ABS_PRESS"},
    {"gsg", "GND_SPEC_GRAV"},
    {"wsp", "WPNAV_SPEED"},
    {"wac", "WPNAV_ACCEL"},
    {"lsp", "LOIT_SPEED"},
    {"lac", "LOIT_ACC_MAX"},

    {"cfx", "COMPASS_OFS_X"},
    {"cfy", "COMPASS_OFS_Y"},
    {"cfz", "COMPASS_OFS_Z"},
	{"cdx", "COMPASS_DIA_X"},
	{"cdy", "COMPASS_DIA_Y"},
	{"cdz", "COMPASS_DIA_Z"},
	{"cox", "COMPASS_ODI_X"},
	{"coy", "COMPASS_ODI_Y"},
	{"coz", "COMPASS_ODI_Z"},
};

static int params_cnt = sizeof(params) / sizeof(params[0]);

bool is_dbg_motor;
bool is_dbg_attitude;
bool is_dbg_ctrl;
uint32_t dbg_print_cnt = 20;
uint32_t dbg_print_timeinterval = 1000;
bool is_dbg_bprintf;

void param_debug_tick(void);
bool is_param_print(void);

static int cmd_param_set(const char *name, float value);
static int cmd_param_show(int argc, char *argv[]);
static void cmd_param(int argc, char *argv[]);
static int cmd_param_reset(void);
static int cmd_param_erase(void);
static void cmd_param_dbg(int argc, char *argv[]);

static void cmd_version(int argc, char *argv[]);
static void cmd_reset(int argc, char *argv[]);
static void cmd_led(int argc, char *argv[]);
static void cmd_mode(int argc, char *argv[]);
static void cmd_ctrl(int argc, char *argv[]);
static void cmd_thr(int argc, char *argv[]);
static void cmd_neo(int argc, char *argv[]);

static int radian_to_degree(float value);
static float degree_to_radian(int value);

AP_HAL::Shell::ShellCommand shell_commands[] = {
    {"param", cmd_param},
    {"version", cmd_version},
    {"reset", cmd_reset},
    {"cali", cmd_cali},
    {"led", cmd_led},
    {"mode", cmd_mode},
    {"ctrl", cmd_ctrl},
    {"thr", cmd_thr},
    {"neo", cmd_neo},
    {NULL, NULL} // this is the end of commands
};

// Notice: argc not inlcude the command string, only params

// param     set     xxx   value
//       argv[0] argv[1] argv[2]
void cmd_param(int argc, char *argv[])
{
    if (argc < 1) { // at least should be param show
        hal.shell->printf("usage: param set|show|dbg|reset|erase [param_short_name value]|dbg_param\r\n");
        return;
    }
    
    if (!strcmp(argv[0], "show")) // param show
    {
        cmd_param_show(argc - 1, &argv[1]);
        return;
    }

    if (!strcmp(argv[0], "reset")) // param reset
    {
        cmd_param_reset();
        return;
    }

    if (!strcmp(argv[0], "erase")) // param reset
    {
        cmd_param_erase();
        return;
    }

    if (!strcmp(argv[0], "dbg"))
    {
        cmd_param_dbg(argc - 1, &argv[1]);
        return;
    }

    if (!strcmp(argv[0], "sps")) {
        // target_loc.lng = (int32_t)argv[1];
        // target_loc.lat = (int32_t)argv[2];
        // target_loc.alt = 0;
        // hal.shell->printf("alt lng lat = %d %d %d \r\n",  target_loc.alt, target_loc.lng, target_loc.lat);
        pos_set_flag = true;
        hal.shell->printf("pos_set_flag = %d \r\n", pos_set_flag);
        return;
    }

    if (!strcmp(argv[0], "gps")) {
        pos_get_flag = true;
        hal.shell->printf("pos_reset_flag = %d \r\n", pos_get_flag);
        return;
    }

    if (argc < 3) {
        hal.shell->printf("usage: param set param_short_name value\r\n");
        return;
    }

    if (!strcmp(argv[0], "set")) // param set
    {
        int i;
        for (i = 0; i < params_cnt; i++) {
            if (!strcmp(argv[1], params[i].param_short_name)){
                cmd_param_set(params[i].param_name, strtod(argv[2], NULL));
                break;
            }
        }

        if (i == params_cnt) {
            hal.shell->printf("not support: param [%s]\r\n", argv[1]);
        }

        return;
    }  

    hal.shell->printf("not support this command\r\n");
}

void cmd_version(int argc, char *argv[]) {
    AP_FWVersion ver = AP_FWVersion::get_fwverz();

    hal.shell->printf("%s\r\n", ver.fw_string);
}

void cmd_thr(int argc, char *argv[])
{
    if (argc == 0)
    {
        hal.shell->printf("thr: %.2f %d\r\n",
                          sub.pulse_thr, sub.pulse_thn);
    }
    else if (argc == 1)
    {
        sub.pulse_thr = strtof(argv[0], NULL);
    }
    else if (argc == 2)
    {
        sub.pulse_thr = strtof(argv[0], NULL);
        sub.pulse_thn = strtol(argv[1], NULL, 10);
    }
    else
    {
        hal.shell->printf("Usage thr [threshold] [n]");
    }
}

void cmd_neo(int argc, char *argv[])
{
    if (argc == 2)
    {
        char *ptr;
        int32_t led = strtol(argv[0], &ptr, 10);
        if (led >= 1 && led <= 5)
        {
            int32_t color = strtol(argv[1], &ptr, 16);
            if (ptr != argv[1])
            {
                hal.rcout->set_neopixel_rgb_data(6, 1<<(led - 1),
                                                 (color>>16)&0xFF,
                                                 (color>>8)&0xFF,
                                                 (color)&0xFF);
                hal.rcout->neopixel_send();
                return;
            }
        }
    }

    hal.shell->printf("Usage neo (1 ~ 5) (RGB)");
}

void cmd_reset(int argc, char *argv[]) {
	bool hold_in_bootloader = false;

	if(argc && argv[0])
		hold_in_bootloader = strtol(argv[0], NULL, 10) ? true : false;

	hal.shell->printf("system reboot%s\r\n", hold_in_bootloader ? ", hold boot" : "");
	hal.scheduler->delay(500);
    hal.scheduler->reboot(hold_in_bootloader);
}

static void cmd_led(int argc, char *argv[])
{
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
    if(argc >= 2 && strlen(argv[0]) == 1
        && *argv[0] >= '1' && *argv[0] <= '4')
    {
        uint32_t pin = 0;
        switch (*argv[0]) {
            case '1':
#ifdef HAL_GPIO_PIN_LED_1
                pin = HAL_GPIO_PIN_LED_1;
#endif
                break;
            case '2':
#ifdef HAL_GPIO_PIN_LED_2
                pin = HAL_GPIO_PIN_LED_2;
#endif
                break;
            case '3':
#ifdef HAL_GPIO_PIN_LED_3
                pin = HAL_GPIO_PIN_LED_3;
#endif
                break;
            case '4':
#ifdef HAL_GPIO_PIN_LED_4
                pin = HAL_GPIO_PIN_LED_4;
#endif
                break;
        }

        if(strcasecmp(argv[1], "on") == 0)
        {
            palWriteLine(pin, 0);
            return;
        }
        else if(strcasecmp(argv[1], "off") == 0)
        {
            palWriteLine(pin, 1);
            return;
        }
    }

    hal.shell->printf("usage: led 1|2|3|4 on|off\r\n");
#endif
}

static void cmd_mode(int argc, char *argv[])
{
    if(argc >= 1)
    {
        if(strcasecmp(argv[0], "manual") == 0)
        {
            bool r = sub.set_mode(MANUAL, ModeReason::RC_COMMAND);
            hal.shell->printf(r ? "ok\r\n" : "fail\r\n");
            return;
        }
        else if(strcasecmp(argv[0], "stabilize") == 0)
        {
            bool r = sub.set_mode(STABILIZE, ModeReason::RC_COMMAND);
            hal.shell->printf(r ? "ok\r\n" : "fail\r\n");
            return;
        }
        else if(strcasecmp(argv[0], "floor") == 0)
        {
            bool r = sub.set_mode(FLOOR, ModeReason::RC_COMMAND);
            hal.shell->printf(r ? "ok\r\n" : "fail\r\n");
            return;
        }
        else if(strcasecmp(argv[0], "waterline") == 0)
        {
            bool r = sub.set_mode(WATERLINE, ModeReason::RC_COMMAND);
            hal.shell->printf(r ? "ok\r\n" : "fail\r\n");
            return;
        }
        else if(strcasecmp(argv[0], "regular") == 0)
        {
            bool r = sub.set_mode(REGULAR, ModeReason::RC_COMMAND);
            hal.shell->printf(r ? "ok\r\n" : "fail\r\n");
            return;
        }
    }
    else
    {
        hal.shell->printf("mode= %d\r\n", sub.control_mode);
    }

    hal.shell->printf("usage: mode [manual | stabilize]\r\n");
}

static void cmd_ctrl(int argc, char *argv[])
{
    if (argc >= 5)
    {
        sub.ctrl_forward = strtof(argv[0], NULL);
        sub.ctrl_yaw = strtof(argv[1], NULL);
        sub.ctrl_left = strtof(argv[2], NULL);
        sub.ctrl_right = strtof(argv[3], NULL);
        sub.ctrl_pump = strtof(argv[4], NULL);
        return;
    }
    else if (argc == 0)
    {
        uint16_t left, right, lp , rp, p;
        SRV_Channels::get_output_pwm(SRV_Channel::k_motor1, left);
        SRV_Channels::get_output_pwm(SRV_Channel::k_motor2, right);
        SRV_Channels::get_output_pwm(SRV_Channel::k_throttleLeft, lp);
        SRV_Channels::get_output_pwm(SRV_Channel::k_throttleRight, rp);
        SRV_Channels::get_output_pwm(SRV_Channel::k_boost_throttle, p);
        hal.shell->printf("PWM: %d %d %d %d %d\r\n",
            left, right, lp, rp, p);
        return;
    }

    hal.shell->printf("usage: ctrl forward yaw left right pump\r\n");
}

// param dbg motor|atti|ctrl on|[off] [print_cnt]
void cmd_param_dbg(int argc, char *argv[]) {
    if (argc <= 0) {
        hal.shell->printf("usage: param dbg motor|atti|ctrl on|[off] [print_cnt]\r\n");
        return;
    }

    if (!strcmp(argv[0], "motor")) {
        if (argc >= 2 && !strcmp(argv[1], "on")) {
            is_dbg_motor = true;
        } else {
            is_dbg_motor = false;
        }
    } else if (!strcmp(argv[0], "atti")) {
        if (argc >= 2 && !strcmp(argv[1], "on")) {
            is_dbg_attitude = true;
        } else {
            is_dbg_attitude = false;
        }
    } else if (!strcmp(argv[0], "ctrl")) {
        if (argc >= 2 && !strcmp(argv[1], "on")) {
            is_dbg_ctrl = true;
        } else {
            is_dbg_ctrl = false;
        }
    } else {
        hal.shell->printf("usage: param dbg motor|atti|ctrl on|[off] [print_cnt]\r\n");
        return;
    }

    if (argc >= 3) {
        dbg_print_cnt = strtol(argv[2], NULL, 10);
    } else {
        dbg_print_cnt = 20;
    }

    hal.shell->printf("set is_dbg_motor %d is_dbg_attitude %d is_dbg_ctrl %d dbg_print_cnt %d.\r\n", 
                is_dbg_motor, is_dbg_attitude, is_dbg_ctrl, dbg_print_cnt);
}

int cmd_param_set(const char *name, float value)
{
    enum ap_var_type var_type;

    // set parameter
    AP_Param *vp;
    char key[AP_MAX_NAME_SIZE+1];
    strncpy(key, (char *)name, AP_MAX_NAME_SIZE);
    key[AP_MAX_NAME_SIZE] = 0;

    hal.shell->printf("param pid set(%s, %3.6f).\r\n", name, value);

    // find existing param so we can get the old value
    vp = AP_Param::find(key, &var_type);
    if (vp == NULL) {
        hal.shell->printf("param name %s does not exist.\r\n", name);
        return -1;
    }
    float old_value = vp->cast_to_float(var_type);

    if (!strcmp((const char*)key, "AHRS_TRIM_X") ||
            !strcmp((const char*)key, "AHRS_TRIM_Y") ||
            !strcmp((const char*)key, "AHRS_TRIM_Z")) {
        value = degree_to_radian(value);
    }

    // set the value
    vp->set_float(value, var_type);

    /*
      we force the save if the value is not equal to the old
      value. This copes with the use of override values in
      constructors, such as PID elements. Otherwise a set to the
      default value which differs from the constructor value doesn't
      save the change
     */
    bool force_save = !is_equal(value, old_value);

    // save the change
    vp->save(force_save);
    return 0;
}

int cmd_param_show(int argc, char *argv[])
{
    enum ap_var_type var_type;
    // set parameter
    AP_Param *vp;
    char key[AP_MAX_NAME_SIZE+1];
    int i;

    for (i = 0; i < params_cnt; i++)
    {
        strncpy(key, (char *)params[i].param_name, AP_MAX_NAME_SIZE);
        key[AP_MAX_NAME_SIZE] = 0;

        //printf("param pid set(%s, %3.6f).\r\n", name, value);

        // find existing param so we can get the old value
        vp = AP_Param::find((const char*)key, &var_type);
        if (vp == NULL) {
            hal.shell->printf("param name %s does not exist.\r\n", key);
            continue;
        }

        float value = vp->cast_to_float(var_type);

        if (argc <= 0 || strcmp(argv[0], "radian")) {
            if (!strcmp((const char*)key, "AHRS_TRIM_X") ||
            !strcmp((const char*)key, "AHRS_TRIM_Y") ||
            !strcmp((const char*)key, "AHRS_TRIM_Z")) {
                value = radian_to_degree(value);
            }
        }

        hal.shell->printf("%s: %3.6f\r\n", key, value);

    }

    return 0;
}

int cmd_param_reset(void) 
{
    //AP_Param::erase_all();
	
    AP_Param::set_and_save_by_name("SYSID_SW_MREV", Parameters::k_format_version + 1);
    hal.shell->printf("parameters reset...\r\n");

    hal.scheduler->delay(500);

    hal.scheduler->reboot(false);

    return 0;
}

int cmd_param_erase(void) 
{
    AP_Param::set_and_save_by_name("SYSID_SW_MREV", 0);
    hal.shell->printf("parameters erase...\r\n");

    hal.scheduler->delay(500);

    hal.scheduler->reboot(false);

    return 0;
}

void param_debug_tick(void)
{
    static uint32_t last_ran_overtime = 0;
    
    if ((AP_HAL::millis() - last_ran_overtime) > dbg_print_timeinterval)
    {
        is_dbg_bprintf = true;
        last_ran_overtime = AP_HAL::millis();
        if (dbg_print_cnt)
        {
            dbg_print_cnt--;
        }
    }
    else
    {
        is_dbg_bprintf = false;
    }

    return;
}

bool is_param_print(void)
{
    return is_dbg_bprintf && (dbg_print_cnt > 0);
}

int radian_to_degree(float value) {
    return (int)(value * 18000 / M_PI);
}

float degree_to_radian(int value) {
    return (float)value * M_PI / 18000;
}