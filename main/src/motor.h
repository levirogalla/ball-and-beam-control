#ifndef MOTOR_H
#define MOTOR_H

void motor_config(int pwm_pin);

void motor_init();

void set_motor_voltage_no_stick(float volts);

void calculate_overshoot_and_settle(void);

#endif