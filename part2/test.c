#include<fcntl.h>
#include<stdio.h>
#include<unistd.h>
#include<stdlib.h>
#include<string.h>
#include<sys/ioctl.h>
#include<time.h>
#include<pthread.h>
#include<sched.h>
#include "hcsr04_structures.h"

/*Circular Buffer to keep track of the measurements*/
typedef struct circular_buffer{
    long distance;
    unsigned long long timestamp;
} circular_buffer;

void *performOperation(void *arg){
    printf("Thread creation Successful. Performing Operations... \n");
    circular_buffer buf;
    int writeInput;
    int fd = (int) arg;
    
    /*Requesting to take the first measurement*/
    /*Any non-zero value */
    // First Write Operation
    writeInput = 1;
    if(write(fd, &writeInput, sizeof(int))){
        printf("Write 1: Unable to take measurement \n");
    }else{
        printf("Write 1: Write Successful \n");
    }

    // Second Operation (Failure Case. It should probably return negative because it will hinder the previous write operation)
    writeInput = 0;
    if(write(fd, &writeInput, sizeof(int))){
        printf("Write 2: Unable to take measurement \n");
    }else{
        printf("Write 2: Write Successful \n");
    }

    // Third Write Operation
    sleep(2);
    writeInput = 1;
    if(write(fd, &writeInput, sizeof(int))){
        printf("Write 3: Unable to take measurement \n");
    }else{
        printf("Write 3: Write Successful \n");
    }

    // Fourth Write Operation
    sleep(2);
    writeInput = 1;
    if(write(fd, &writeInput, sizeof(int))){
        printf("Write 4: Unable to take measurement \n");
    }else{
        printf("Write 4: Write Successful \n");
    }
    
    // Fifth Write Operation
    sleep(2);
    writeInput = 1;
    if(write(fd, &writeInput, sizeof(int))){
        printf("Write 5: Unable to take measurement \n");
    }else{
        printf("Write 5: Write Successful \n");
    }

    /*Reading the Measured data (Read directly from buffer) */
    if(read(fd, &buf, sizeof(buf)) < 0){ 
        printf("\t\t\t\t\t***Unable to retrive distance***\n");
    }else{
        printf("Reading data 1: distance-%ldcms \t\t timestamp-%lld\n", buf.distance, buf.timestamp);
    }

    /*Reading the Measured data (Read directly from buffer) */
    if(read(fd, &buf, sizeof(buf)) < 0){ 
        printf("\t\t\t\t\t***Unable to retrive distance***\n");
    }else{
        printf("Reading data 2: distance-%ldcms \t\t timestamp-%lld\n", buf.distance, buf.timestamp);
    }

    /*Reading the Measured data (Read directly from buffer) */
    if(read(fd, &buf, sizeof(buf)) < 0){ 
        printf("\t\t\t\t\t***Unable to retrive distance***\n");
    }else{
        printf("Reading data 3: distance-%ldcms \t\t timestamp-%lld\n", buf.distance, buf.timestamp);
    }

    /*Reading the Measured data */
    if(read(fd, &buf, sizeof(buf)) < 0){ 
        printf("\t\t\t\t\t***Unable to retrive distance***\n");
    }else{
        printf("Reading data 4: distance-%ldcms \t\t timestamp-%lld\n", buf.distance, buf.timestamp);
    }
    
    /*Reading the Measured data */
    if(read(fd, &buf, sizeof(buf)) < 0){ 
        printf("\t\t\t\t\t***Unable to retrive distance***\n");
    }else{
        printf("Reading data 5: distance-%ldcms \t\t timestamp-%lld\n", buf.distance, buf.timestamp);
    }

    /*Reading the Measured data */
    if(read(fd, &buf, sizeof(buf)) < 0){ 
        printf("\t\t\t\t\t***Unable to retrive distance***\n");
    }else{
        printf("Reading data 6: distance-%ldcms \t\t timestamp-%lld\n", buf.distance, buf.timestamp);
    }
    
    /*Reading the Measured data */
    if(read(fd, &buf, sizeof(buf)) < 0){ 
        printf("\t\t\t\t\t***Unable to retrive distance***\n");
    }else{
        printf("Reading data 7: distance-%ldcms \t\t timestamp-%lld\n", buf.distance, buf.timestamp);
    }

    printf("Thread Exiting");
    pthread_exit(NULL);
}

int main(int argc, char **argv){
     /*Thread for each device */
    pthread_t thread_dev1;
    // pthread_t thread_dev2;
    config_input *conf_input = (config_input *)malloc(sizeof(config_input));
    setparam_input *setp_input = (setparam_input *)malloc(sizeof(setparam_input));
    int return_value; 
    int fd1, fd2; /* File Descriptor*/
    int flag = 1;

    fd1 = open("/dev/HCSR_1", O_RDWR);

    if(fd1 < 0 ){
        printf("Cannot Open Device. Exiting....\n");
        return 0;
    }

    fd2 = open("/dev/HCSR_2", O_RDWR);

    if(fd2 < 0 ){
        printf("Cannot Open Device. Exiting....\n");
        return 0;
    }


    /* Configuring pins through the input*/
    conf_input->echo_pin=5;
    conf_input->trigger_pin=1;
    if(ioctl(fd1, CONFIG_PINS, conf_input) < 0){
        printf("\t\t\t\t\t***Unable to configure pins for device 1*** \n");
        flag = 0;
    }

    /*Configuring set up parameters*/
    setp_input->num_samples=8;
    setp_input->sampling_period=80;
    if(ioctl(fd1, SET_PARAMETERS, setp_input) < 0){
        printf("\t\t\t\t\t***Unable to set parameters for device 1*** \n");
        flag = 0;
    }


    /*Creating thread with the given attirbute and scheduling policy to perform the device operations*/
    if(flag){
        return_value = pthread_create(&thread_dev1, NULL, performOperation, (void *)fd1);
    }else{
        printf("Base configuration failed for device 1 \n");
    }


    if(return_value){
        printf("Error in creating thread for device 1\n");
        return 0;
    }

    // flag = 1;
    
    // sleep(5);
    // /* Configuring pins through the input*/
    // conf_input->echo_pin=11;
    // conf_input->trigger_pin=4;
    // if(ioctl(fd2, CONFIG_PINS, conf_input) < 0){
    //     printf("\t\t\t\t\t***Unable to configure pins for device 2*** \n");
    //     flag = 0;
    // }

    // /*Configuring set up parameters*/
    // setp_input->num_samples=8;
    // setp_input->sampling_period=80;
    // if(ioctl(fd2, SET_PARAMETERS, setp_input) < 0){
    //     printf("\t\t\t\t\t***Unable to set parameters for device 2*** \n");
    //     flag = 0;
    // }

    // if(flag){
    //     return_value = pthread_create(&thread_dev2, NULL, performOperation, (void *)fd2);
    // }else{
    //        printf("Base configuration failed for device 2 \n");
    // }

    // if(return_value){
    //     printf("Error in creating thread for device 2\n");
    //     return 0;
    // }

    pthread_join(thread_dev1, NULL);
    // pthread_join(thread_dev2, NULL);
   
    close(fd1);
    close(fd2);
    return 0;
}