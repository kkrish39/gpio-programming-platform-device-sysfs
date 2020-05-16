# gpio-programming-platform-device-sysfs


# About the project
The main aim of the project is to establish an interface between the Intel Galelio Gen II board and a series of Ultrasonic sensors(HC-SR04) in measuring the distance for a range of 4 metres.

The project consist of two parts. The first part deals with the character devices and the second part deals in establishing the communication through sysfs file system.

I've followed an incremental development approach. Part 1 will run on it's own with specific test file. Extending the part 1, I've done some tweaks in-order to support part 2 with the test file and test_script, to test both the parts.

I've disabled the debug information, and printing only the important information as ALERT which can as well be controlled by adding/removing the macros.

# Part1

Commands to execute:
 - Compile the program using "make" command.
 - Move the hcsr04_drv.ko and hcsr04_tester file to the board.
 - Execute "insmod hcsr04_drv.ko n_gpio = x". x -> is the required number of gpio devices that needs to be registered.
 - To perform test, run "./hcsr04_tester".

# Steps executed in the test file:

**Sample Output: (Moved the object while measuring. So there is a difference in length measured)**

**USER OUTPUT**

Thread creation Successful. Performing Operations... 
Write 1: Write Successful 
Write 2: Unable to take measurement 
Write 3: Write Successful 
Write 4: Write Successful 
Write 5: Write Successful 
Reading data 1: distance-9cms 		 timestamp-87006366600
Reading data 2: distance-17cms 		 timestamp-87812552936
Reading data 3: distance-17cms 		 timestamp-88618679132
Reading data 4: distance-14cms 		 timestamp-89416822614
Reading data 5: distance-22cms 		 timestamp-90075294288
Reading data 6: distance-15cms 		 timestamp-90721813536
Reading data 7: distance-11cms 		 timestamp-91368257698
******* Thread Exiting *********

**KERNEL OUTPUT**

[  148.565433] ALERT:Requested PIN already set as echo for this specific device 
[  148.572731] ALERT:Requested PIN already set as trigger for this specific device 
[  148.584473] ALERT:Triggering measurment from Write Function 
[  148.594251] ALERT:**There is an Ongoing Measurement**
[  150.190174] ALERT:#SUM: 160 		
[  150.193092] ALERT:#AVERAGE_DISTANCE: 20 
[  150.602967] ALERT:Triggering measurment from Write Function 
[  152.200172] ALERT:#SUM: 97 		
[  152.203003] ALERT:#AVERAGE_DISTANCE: 12 
[  152.612369] ALERT:Triggering measurment from Write Function 
[  154.210174] ALERT:#SUM: 123 		
[  154.213093] ALERT:#AVERAGE_DISTANCE: 15 
[  154.621739] ALERT:Triggering measurment from Write Function 
[  156.230256] ALERT:#SUM: 118 		
[  156.233174] ALERT:#AVERAGE_DISTANCE: 14 
[  156.238544] ALERT:About to send distace:20 Timestamp:69148097758 
[  156.248182] ALERT:About to send distace:12 Timestamp:69950182764 
[  156.260594] ALERT:About to send distace:15 Timestamp:70752341638 
[  156.272125] ALERT:About to send distace:14 Timestamp:71558487718 
[  156.283558] ALERT:Triggering measurment from Read Function 
[  157.880171] ALERT:#SUM: 78 		
[  157.883001] ALERT:#AVERAGE_DISTANCE: 9 
[  157.887124] ALERT:About to send distace:9 Timestamp:72216844170 
[  157.894713] ALERT:Triggering measurment from Read Function 
[  159.510165] ALERT:#SUM: 130 		
[  159.513083] ALERT:#AVERAGE_DISTANCE: 16 
[  159.517293] ALERT:About to send distace:16 Timestamp:72867390730 
[  159.526773] ALERT:Triggering measurment from Read Function 
[  161.130158] ALERT:#SUM: 124 		
[  161.133076] ALERT:#AVERAGE_DISTANCE: 15 
[  161.137285] ALERT:About to send distace:15 Timestamp:73513876980 
[  161.148565] ALERT:HCSR_1 is closing 
[  161.153101] ALERT:HCSR_2 is closing 
[  193.314338] ALERT:HCSR_1 is opening 
[  193.318034] ALERT:HCSR_2 is opening 

# PART 2

# Commands to execute:
 - Compile the program using "make" command.
 - Move PHCSR04_device.ko, PHCSR04_driver.ko, hcsr04_tester and test_script.sh files to the board.
 - Execute "insmod PHCSR04_device.ko" and "insmod PHCSR04_driver.ko". [This can be in any order]
 - Make the script executable by executing the following command "chmod +x test_script.sh"
 - To perform test, run "./hcsr04_tester" or test_script.sh.  

# About the test file:
 - Handled the case to avoid error for duplicate registration of pins from the same device. So test file can be ran multiple times along and along with the test script as well.


# Sample Output: (I first ran ./hcsr04_tester and test_script.sh with moving objects)
(Distance will return -1 if the buffer is empty. enable 1 will trigger one more measurement and after reading that will give us measured value)

**USER OUTPUT**
root@quark:/# insmod PHCSR04_device.ko 
root@quark:/# insmod PHCSR04_driver.ko 
root@quark:/# ./hcsr04_tester 
Thread creation Successful. Performing Operations... 
Write 1: Write Successful 
Write 2: Unable to take measurement 
Write 3: Write Successful 
Write 4: Write Successful 
Write 5: Write Successful 
Reading data 1: distance-16cms 		 timestamp-31268392278
Reading data 2: distance-9cms 		 timestamp-32077007544
Reading data 3: distance-19cms 		 timestamp-32866007206
Reading data 4: distance-10cms 		 timestamp-33663274962
Reading data 5: distance-13cms 		 timestamp-34301282882
Reading data 6: distance-21cms 		 timestamp-34939186804
Reading data 7: distance-12cms 		 timestamp-35576879722
Thread Exitingroot@quark:/# ./test_script.sh 
Trigger Pin Set: 
1
Echo Pin Set: 
5
Number of samples: 
8
Sampling period: 
70
Please wait...Writing 4 more values
Reading distance
12
Reading distance
13
Reading distance
15
Reading distance
7
Reading distance
-1
Please wait...Writing 1 more value
Reading distance
12

**KERNEL OUTPUT**
[   47.448472] Device HCSR_1 registered 
[   47.469651] Device HCSR_2 registered 
[   51.345949] INFO:1Device found HCSR_1 20 
[   51.357130] INFO:Class got registered Successfully 
[   51.370290] INFO:Miscellaneous class registration successful 
[   51.385574] INFO:Device creation Successful 
[   51.391362] INFO:Device got binded with the driver 
[   51.396501] INFO:1Device found HCSR_2 55 
[   51.400781] INFO:Class got registered Successfully 
[   51.415268] INFO:Miscellaneous class registration successful 
[   51.426836] INFO:Device creation Successful 
[   51.432550] INFO:Device got binded with the driver 
[   53.699734] INFO:HCSR_1 is opening 
[   53.703474] INFO:HCSR_2 is opening 
[   53.716919] INFO:**There is an Ongoing Measurement**
[   66.124582] INFO:HCSR_1 is closing 
[   66.128143] INFO:HCSR_2 is closing 
[   70.525661] INFO:Requested PIN already set as trigger for this specific device 
[   70.533150] INFO:Trigger Pin has been Set to 1 
[   72.564087] INFO:Requested PIN already set as echo for this specific device 
[   72.571330] INFO:Echo Pin has been Set to 5 
[   87.034268] INFO:There is no distance measures to read. Please enable device for new measurement.
