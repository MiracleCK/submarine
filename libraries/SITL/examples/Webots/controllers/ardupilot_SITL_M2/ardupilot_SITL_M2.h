// #define DEBUG_MOTORS 
// #define DEBUG_WIND    
// #define DEBUG_SENSORS   
// #define DEBUG_USE_KB 
// #define DEBUG_INPUT_DATA
// #define LINEAR_THRUST
#define WIND_SIMULATION

#define VEHICLE_DRAG_FACTOR 0.001

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))


enum data_type {
        DATA_FLOAT,
        DATA_DOUBLE,
        DATA_VECTOR4F,
        DATA_VECTOR8F,
    };

struct vector4f 
{
    float w,x,y,z;
};

struct vector8f 
{
    float w,x,y,z,a,b,c,d;
};

typedef struct vector4f VECTOR4F; 
typedef struct vector8f VECTOR8F;

struct {
        double timestamp;
        VECTOR8F motors;
        VECTOR4F wind; 
        /*
         struct {
        float speed;      // m/s
        float direction;  // degrees 0..360
        float turbulence;
        float dir_z;	  //degrees -90..90
        } wind;
        */
       } state, last_state;



// table to aid parsing of JSON sensor data
struct keytable {
        const char *section;
        const char *key;
        void *ptr;
        enum data_type type;

} keytable[2] = {
        //{ "", "timestamp", &state.timestamp, DATA_DOUBLE },
        { "", "eng",    &state.motors, DATA_VECTOR8F },
        { "", "wnd",    &state.wind, DATA_VECTOR4F }
};

/*
        w: wind speed
        x , y, z: wind direction.
*/
VECTOR4F __attribute__((packed, aligned(1)))  wind_webots_axis;

