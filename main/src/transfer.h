#ifndef TRANSFER_H
#define TRANSFER_H

#define MOTOR_CONTROLLER_KP -28

float pot_angle_raw_to_rad(int v);

float sense_distance_raw_to_m(int v);

float motor_controller_theta_to_volt_0(float theta_target, float theta);

float motor_controller_theta_to_volt_1(float theta_target, float theta,
                                       float *previous_state);

// Initialize both history values to 0 and retain them between control updates.
float ball_controller_position_to_motor_angle(float position_target,
                                              float position,
                                              float *previous_output,
                                              float *previous_error);

void motor_controller_enable(bool enable);

bool motor_controller_enabled();

void ball_controller_enable(bool enable);

bool ball_controller_enabled();

#endif
