/*
 * Copyright (c) 2021, Andrew Capatina
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
/**
 *  Andrew Capatina
 *  06/19/2019
 * 
 *  Ad-hoc defense.
 * 
 *  Header file for the PCA9685 driver.
 * 
 * */

// Registers.
#define PCA9685_MODE1 0x0
#define PCA9685_MODE2 0x1
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x6           // Responsible for PWM outputs to servo.

#define SERVO_UP_CH 0           // Upper servo channel.
#define SERVO_DOWN_CH 1         // Lower servo channel.

#define SERVO_MIN_PULSE 81      // Minimum pulse written to LED0_ON_L. 
#define SERVO_MAX_PULSE 574     // Maximum pulse written to LED0_ON_L.

#define MIN_DEGREE 0            // Minimum degree by application.
#define MAX_DEGREE 180          // Maximum degree by application.

extern int addr_global;       // Current slave address.
extern int adapter_nr_global;     // I2C device number given by i2cdetect.

/**
 * Serial interface functions.
 * */
int i2c_close(int file);
int i2c_init(int adapter_nr, int addr);

/**
 * PCA9685 driver functions.
 * */
int PCA9685_init(int file);
int PCA9685_reset(int file);
int PCA9685_set_pwm(int file, __uint8_t channel, __uint16_t on, __uint16_t off);
int PCA9685_set_pwm_freq(int file, float freq);
int PCA9685_set_servo_degree(int file, __uint8_t n, __uint8_t degree);

/**
 * Register read and write functions.
 * */
int rd_sensor_reg8(int file,__uint8_t reg_id,__uint8_t * reg_data);
int wr_sensor_reg8(int file,__uint8_t reg_id, __uint8_t reg_dat);