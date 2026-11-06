#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

void motor_config(int pwm_pin, int dir_pin);

int motor_init();

void motor_voltage_no_stick_set(float volts);

void theta_desired_set(float theta);

float theta_desired_get(void);

float theta_desired_saturated_get(void);

void calculate_stiction(void);

void calculate_overshoot(uint32_t period_ms);

void calculate_sine_wave(uint32_t period_ms);

#endif