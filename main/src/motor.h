#ifndef MOTOR_H
#define MOTOR_H

void motor_config(int pwm_pin, int dir_pin);

int motor_init();

void set_motor_voltage_no_stick(float volts);

#endif