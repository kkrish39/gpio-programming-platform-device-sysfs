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

/*Circular Buffer to keep track of the measurements*/
typedef struct circular_buffer{
    long distance;
    unsigned long long timestamp;
} circular_buffer;

/*List of gpio pins that may be associated with the a given I/O pin */
typedef struct pins_to_configure {
    int digitalPin;
    int gpio_pin;   /* GPIO  Pin*/
    int shift_pin;  /*shift pin*/
    int pull;       /*pull pin*/
    int mux1;       /*mux1_pin*/
    int mux2;       /*mux2_pin*/
} pins;
