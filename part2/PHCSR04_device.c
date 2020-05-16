/*
* Reusing the provided sample platform driver program
*/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "PHCSR04.h"

/*A dummy release function*/
static void PHCSR04_device_release(struct device *dev){
	printk("Releasing the device \n");
}

/*Second Device*/
static struct platform_chip device1_chip = {
	.name = "HCSR_1",
	.dev_no = 20,
	.plf_dev = {
		.name = "PHCSR_1",
		.id	= -1,
		.dev = {
			.release = PHCSR04_device_release,
		},
	}
};

/*First device */
static struct platform_chip device2_chip = {
	.name = "HCSR_2",
	.dev_no = 55,
	.plf_dev = {
		.name = "PHCSR_2",
		.id	 = -1,
		.dev = {
			.release = PHCSR04_device_release,
		}
	}
};

/*Function to initilaize the device */
static int PHCSR04_device_init(void){
    /* Registering the device 1 */
	if(platform_device_register(&device1_chip.plf_dev)){
		printk(KERN_ALERT "Device %s registration unsuccessful\n", device1_chip.name);
		return -EINVAL;
	}
	printk(KERN_ALERT "Device %s registered \n", device1_chip.name);

	/* Registering the device 2 */
	if(platform_device_register(&device2_chip.plf_dev)){
		printk(KERN_ALERT "Device %s registration unsuccessful\n", device2_chip.name);
		return -EINVAL;
	}
	printk(KERN_ALERT "Device %s registered \n", device2_chip.name);
	
	return 0;
}

/*Function to handle the device exit*/
static void PHCSR04_device_exit(void){
	/*Unregistering the 2nd device*/
	platform_device_unregister(&device2_chip.plf_dev);
	printk(KERN_ALERT "Unregistering the device: %s \n", device2_chip.name);

	/*Unregistering the 1st device*/
    platform_device_unregister(&device1_chip.plf_dev);
	printk(KERN_ALERT "Unregistering the device: %s \n", device1_chip.name);
	
}


/*Initialize and exit Modules*/
module_init(PHCSR04_device_init);
module_exit(PHCSR04_device_exit);

MODULE_LICENSE("GPL");