/**
 *  Andrew Capatina
 *  Ad-hoc defense
 *  06/19/2021
 * 
 *  Description:
 * 
 *  .cpp file for PCA9685 driver. 
 * 
 *  Contains various functions to drive the servos
 *  connected to the device.
 * 
 *  This implementation uses two channels only.  
 * 
 * */

#include <linux/i2c-dev.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "servo_driver.h"

// Could be dynamically allocated.
int addr_global = 0x40;        // Current slave address.
int adapter_nr_global = 1;     // I2C device number given by i2cdetect.

/*
    Function to close the I2C interface.

    file: file pointer to I2C slave.
*/
int i2c_close(int file)
{
    // Cleaning up I2C device.
    if(close(file) < 0)
    {
        printf("Failed to close I2C device: errno(%s)\n", strerror(errno));

        return -1;
    }

    printf("Cleaned up I2C device, closing now...\n");

    return 0;
}


/*
    Function to initialize the I2C device.

    file: file pointer to I2C slave.
    addr: I2C addr given by i2cdetect().
*/
int i2c_init(int adapter_nr, int addr)
{
    int file;
    char filename[20];
    
    snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);  // Creating I2C adapter string.

    file = open(filename, O_RDWR);  // Specifying which I2C adapter to access.

    if(file < 0)
    {
        printf("Failed to open I2C device: errno(%s)\n", strerror(errno));

        return -1;
    }

    printf("Success opening I2C device!\n");

    if(ioctl(file, I2C_SLAVE, addr) < 0)    // Specifying which device to communicate with.
    {
        printf("Failed to specify I2C address: errno(%s)\n", strerror(errno));
        // Cleaning up I2C device.
        if(close(file) < 0)
        {
            printf("Failed to close I2C device: errno(%s)\n", strerror(errno));

            return -1;
        }

        return -1;
    }

    printf("Success specifying I2C address!\n");
    return file;
}


/*
    The inititlization function of the slave device.

    file: file pointer to I2C slave.
*/
int PCA9685_init(int file)
{
    int rtn;

    rtn = PCA9685_reset(file);

    if(rtn < 0)
    {
        printf("PCA9685_reset(): function failed.\n");
        return -1;
    }

    __uint8_t data;

    // Perform read-modify-write.
    rtn = rd_sensor_reg8(file, PCA9685_MODE2, &data);

    // Setting this bit prevents jitter on the servos.
    data = data | 0x08; // Set the OCH bit.

    rtn = wr_sensor_reg8(file, PCA9685_MODE2, data);

    rtn = PCA9685_set_pwm_freq(file, 60);

    if(rtn < 0)
    {
        printf("PCA9685_setPWMfreq: function failed.\n");
        return -1;
    }

    return 0;
}


/*
    Function to reset the slave device.

    file: file pointer to I2C slave.
*/
int PCA9685_reset(int file) 
{

    int rtn;
    __uint8_t data;

    // 0x80 is the register data for the reset.
    rtn = wr_sensor_reg8(file, PCA9685_MODE1, 0x80);

    usleep(500000);

    if(rtn < 0)
        return -1;

    return 0;
}


/*
    Function to write to the PWM registers of the slave device.
        This allows the servos to turn.

    file: file pointer to I2C slave.
    channel: Set of LED registers selected. (0-1)
    on: time when the PWM output is high.
    off: time when the PWM output shuts off.
*/
int PCA9685_set_pwm(int file, __uint8_t channel, __uint16_t on, __uint16_t off)
{
    int rtn;

    __uint8_t off_l = off & 0x00FF;
    __uint8_t off_h = (off & 0xFF00) >> 8; 

    rtn = wr_sensor_reg8(file, LED0_ON_L+4*channel, on & 0xFF);

    if(rtn < 0)
    {
        printf("PCA9685_set_pwm(): 1st write failed.\n");
        return -1;
    }

    rtn = wr_sensor_reg8(file, LED0_ON_L+4*channel+1, on >> 8);

    if(rtn < 0)
    {
        printf("PCA9685_set_pwm(): 2nd write failed.\n");
        return -1;
    }

    rtn = wr_sensor_reg8(file, LED0_ON_L+4*channel+2, off_l);

    if(rtn < 0)
    {
        printf("PCA9685_set_pwm(): 3rd write failed.\n");
        return -1;
    }

    rtn = wr_sensor_reg8(file, LED0_ON_L+4*channel+3, off_h);

    if(rtn < 0)
    {
        printf("PCA9685_set_pwm(): 4th write failed.\n");
        return -1;
    }

    return 0;
}


/*
    Function to set the PWM frequency of slave device.

    file: file pointer to I2C slave.
    freq: frequency in Hz to set the slave at.

*/
int PCA9685_set_pwm_freq(int file, float freq)
{
    int rtn;

    freq *= 0.9;
    float prescalval = 25000000;
    prescalval /= 4096;
    prescalval /= freq;
    prescalval -= 1;
    
    __uint8_t prescale = floor(prescalval + 0.5);

    __uint8_t oldmode;

    rtn = rd_sensor_reg8(file, PCA9685_MODE1, &oldmode);

    if(rtn < 0)
    {
        printf("PCA9685_setPWMfreq(): Failed to read. (1) \n");
        return -1;
    }

    __uint8_t newmode = (oldmode & 0x7F) | 0x10;

    rtn = wr_sensor_reg8(file, PCA9685_MODE1, newmode);

    if(rtn < 0)
    {
        printf("PCA9685_setPWMfreq(): Failed to write. (2) \n");
        return -1;
    }

    rtn = wr_sensor_reg8(file, PCA9685_PRESCALE, prescale);

    if(rtn < 0)
    {
        printf("PCA9685_setPWMfreq(): Failed to write. (3) \n");
        return -1;
    }

    rtn = wr_sensor_reg8(file, PCA9685_MODE1, oldmode);

    if(rtn < 0)
    {
        printf("PCA9685_setPWMfreq(): Failed to write. (4) \n");
        return -1;
    }

    usleep(500000);

    return 0;
}


/*
    Function to translate degree to a PWM pulse.

    Variable zero indicates the time step in which the output 
    turns on. The pulse variable indicates when the output will 
    be shut off.


    file: file pointer to I2C slave.
    n: Channel of LED registers.
    degree: location of servo.
*/
int PCA9685_set_servo_degree(int file, __uint8_t n, __uint8_t degree)
{
    int rtn;
    __uint16_t zero = 0;    // Zero because the on time begins at t=0.

    if(degree > MAX_DEGREE)
    {
        degree = MAX_DEGREE;
    }
    else if(degree < MIN_DEGREE)
    {
        degree = MIN_DEGREE;
    }

    double slope = (574.00 - 81.00) / (180.00 - 0.00);
    __uint16_t pulse = 81.00 + slope * (degree - 0.00);

    // Range is from about 2% 81 of 4096 (lower) to 14% 574 (upper) @60 Hz freq.
    rtn = PCA9685_set_pwm(file, n, zero, pulse);

    if(rtn < 0)
    {
        printf("set_servo_degree(): set_pwm() failed.\n");
        return -1;
    }

    return 0;
}


/*
    Function to read a byte from specified register.

    file: file pointer to I2C slave.
    reg_id: register to read from.
    reg_data: data to be filled by pass by reference.
*/
int rd_sensor_reg8(int file,__uint8_t reg_id,__uint8_t * reg_data)
{
    usleep(10000);

    if(write(file, &reg_id, 1) != 1)
    {
        printf("rd_sensor_reg8(): Write failed.\n");
        return -1;
    }

    usleep(10000);

    if(read(file, reg_data, 1) != 1)
    {
        printf("rd_sensor_reg8(): Read failed.\n");
        return -1;
    }

    usleep(10000);

    return 0;
}


/*
    Function to write a byte to register.

    file: file pointer to I2C slave.
    reg_id: Register address to write to.
    reg_data: data to be written at reg_id.
*/
int wr_sensor_reg8(int file,__uint8_t reg_id, __uint8_t reg_dat)
{
    __uint8_t buf[4];

    buf[0] = reg_id;
    buf[1] = reg_dat;
    
    usleep(10000);

    int rtn = write(file, buf, 2);

    
    if(rtn != 2){

        printf("wr_sensor_reg8(): Write failed. rtn = %i\n", rtn);
        printf("errno : %s \n", strerror(errno));
        return -1;
    }

    return 0;
}
