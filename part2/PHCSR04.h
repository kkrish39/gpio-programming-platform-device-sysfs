#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>
#include <linux/cdev.h>
#include "fifo.h"

/* Platform chip structure */
struct platform_chip {
    char *name; /*Chip Name */
    int	dev_no; /*Major Number */
    struct platform_device 	plf_dev; /*  */
    struct hcsr04_dev *hcsr04; /* device structure to perform and keep track of measurements  */
    struct device *hcsr04_device; /* keep tract of associated device */
};

/*Circular Buffer to keep track of the measurements*/
typedef struct circular_buffer{
    long distance; /*distance measure */
    unsigned long long timestamp; /*timestamp when the measure was recorded */
} circular_buffer;

/*List of gpio pins that may be associated with the a given I/O pin */
typedef struct pins_to_configure {
    int digitalPin;
    int gpio_pin;   /* GPIO pin */
    int shift_pin;  /* Shift pin */
    int pull;       /* Pull pin  */
    int mux1;       /* MUX1 pin  */
    int mux2;       /* MUX2 pin  */
} pins;

/*Work queue structure to queue the work*/
struct work_queue {
	struct work_struct work;
	struct hcsr04_dev *parameter;
} work_queue;

/* Per device structure */
struct hcsr04_dev {
	char name[20]; /* Name of the device */
    struct miscdevice misc_device; /* Miscellaneous character driver*/
    pins echo_pin; /*pin structure to keep track of related gpio pins for echo pin*/
    pins trigger_pin; /*pin structure to keep track of related gpio pins for echo pin*/
    unsigned long long time_stamp; /*time when the distance is recorded */
    int distance;/*The distance measure in centimeter*/
    int num_samples_per_measurement;/*Number of samples per  measurement */
    int sampling_period;/*Sampling period*/
    unsigned long long trig_time; /*Time when the Trigger pin is triggered*/
    unsigned long long echo_time; /*Time when the echo pin receives an echo*/
    int irq_number; /*IRQ number that is being generated for the given echo pin*/
    struct work_queue *test_wq; /*Workqueue specific for each device */
    struct mutex lock, sampleRunning; /*per-device locks to enforce synchronization*/
    int isWorkInitialized; /*Flag to keep track whether workQueue is initialized*/
    int isEnabled; /*Keep track of whether measurement is enabled or not*/
    circular_buf *list; /*FIFO buffer to store data*/
};