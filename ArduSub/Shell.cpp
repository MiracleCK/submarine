
#include <string.h>

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

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
    {"pzp", "POS_Z_P"},
    {"vzp", "VEL_Z_P"},
    {"azp", "ACCEL_Z_P"},
    {"arp", "ATC_ANG_RLL_P"},
    {"app", "ATC_ANG_PIT_P"},
    {"ayp", "ATC_ANG_YAW_P"},
    {"arfe", "ATC_RATE_FF_ENAB"},
    {"azi", "ACCEL_Z_I"},
    {"azd", "ACCEL_Z_D"},
    {"m1d", "MOT_1_DIRECTION"},
    {"m2d", "MOT_2_DIRECTION"},
    {"m3d", "MOT_3_DIRECTION"},
    {"m4d", "MOT_4_DIRECTION"},
    {"m5d", "MOT_5_DIRECTION"},
    {"m6d", "MOT_6_DIRECTION"},
    {"m7d", "MOT_7_DIRECTION"},
    {"m8d", "MOT_8_DIRECTION"}
};

static int params_cnt = sizeof(params) / sizeof(params[0]);

static int cmd_param_set(const char *name, float value);
static int cmd_param_show(void);
static void cmd_param(int argc, char *argv[]);
static int cmd_param_reset(void);

AP_HAL::Shell::ShellCommand shell_commands[] = {
    {"param", cmd_param},
    {NULL, NULL} // this is the end of commands
};

// Notice: argc not inlcude the command string, only params

// param     set     xxx   value
//       argv[0] argv[1] argv[2]
void cmd_param(int argc, char *argv[]) 
{
    if (argc < 1) { // at least should be param show
        hal.shell->printf("usage: param set|show [param_short_name value]\r\n");
        return;
    }
    
    if (!strcmp(argv[0], "show")) // param show
    {
        cmd_param_show();
        return;
    }

    if (!strcmp(argv[0], "reset")) // param reset
    {
        cmd_param_reset();
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

int cmd_param_show(void)
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

        hal.shell->printf("%s: %3.6f\r\n", key, value);

    }

    return 0;
}

int cmd_param_reset(void) {
    AP_Param::erase_all();
    hal.shell->printf("All parameters reset, would auto-reboot board now\r\n");

    hal.scheduler->delay(500);

    hal.scheduler->reboot(false);

    return 0;
}
