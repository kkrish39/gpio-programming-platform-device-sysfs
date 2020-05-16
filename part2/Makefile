CC = i586-poky-linux-gcc
ARCH = x86
CROSS_COMPILE = i586-poky-linux-
SDKTARGETSYSROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux
export PATH:=/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin:/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux:$(PATH)

APP = hcsr04_tester

obj-m:= PHCSR04_driver.o PHCSR04_device.o

all:
	make EXTRA_FLAGS=-g -Wall ARCH=x86 CROSS_COMPILE=i586-poky-linux- -C $(SDKTARGETSYSROOT)/usr/src/kernel M=$(PWD) modules
	$(CC) -o $(APP) test.c -g -Wall -lpthread --sysroot=$(SDKTARGETSYSROOT)

clean:
	rm -f *.ko
	rm -f *.o
	rm -f Module.symvers
	rm -f modules.order
	rm -f *.mod.c
	rm -rf .tmp_versions
	rm -f *.mod.c
	rm -f *.mod.o
	rm -f \.*.cmd
	rm -f Module.markers
	rm -f $(APP) 
