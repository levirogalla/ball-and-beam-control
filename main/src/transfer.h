#ifndef TRANSFER_H
#define TRANSFER_H

#define MOTOR_CONTROLLER_KP -28

float pot_angle_raw_to_rad(int v);

float motor_controller_theta_to_volt_0(float theta_target, float theta);

float motor_controller_theta_to_volt_1(float theta_target, float theta,
                                       float *previous_state);

#endif
