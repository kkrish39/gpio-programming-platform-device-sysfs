#include <linux/ioctl.h>

/*IOCTL commands to set parameters and configure pins*/
#define CONFIG_PINS _IOWR('r',0, config_input)
#define SET_PARAMETERS _IOWR('d',0, setparam_input)

/* Input structrue to configure pins for a device */
typedef struct config_input{
    unsigned trigger_pin;
    unsigned echo_pin;
} config_input;

/* Input structrue to change the metrics of data measurement */ 
typedef struct setparam_input{
    int num_samples;
    int sampling_period;
} setparam_input;