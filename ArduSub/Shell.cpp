
#include <string.h>

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

typedef struct {
    char* param_short_name;
    char* param_name;
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
    {"azd", "ACCEL_Z_D"}
};

static int params_cnt = sizeof(params) / sizeof(params[0]);

static int cmd_param_set(const char *name, float value);
static int cmd_param_show(void);
static void cmd_param(int argc, char *argv[]);

AP_HAL::Shell::ShellCommand shell_commands[] = {
    {"param", cmd_param},
    {NULL, NULL} // this is the end of commands
};

void cmd_param(int argc, char *argv[]) 
{
    float value;

    if (argc < 2) {
        hal.shell->printf("should be param [set|show] [param_short_name]\r\n");
        return;
    }
    
    if (!strcmp(argv[1], "show")) 
    {
        cmd_param_show();
        return;
    }

    if (!strcmp(argv[1], "set"))
    {
        if (argc < 3) {
            hal.shell->printf("not support: need param name\r\n");
        }

        value = strtod(argv[3], NULL);
        int i;
        for (i = 0; i < params_cnt; i++) {
            if (!strcmp(argv[2], params[i].param_short_name)){
                cmd_param_set(params[i].param_name, value);
            }
        }

        if (i == params_cnt) {
            hal.shell->printf("not support: param %s\r\n", argv[2]);
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
            return -1;
        }

        float value = vp->cast_to_float(var_type);

        hal.shell->printf("%s: %3.6f\r\n", key, value);

    }

    return 0;
}
