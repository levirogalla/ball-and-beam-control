#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

void motor_config(int pwm_pin, int dir_pin);

int motor_init();

void set_motor_voltage_no_stick(float volts);

void set_theta_desired(float theta);

float get_theta_desired(void);

void calculate_overshoot(uint32_t period_ms);

#endif