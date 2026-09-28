#ifndef TRANSFER_H
#define TRANSFER_H

float pot_angle_raw_to_rad(int v);

float motor_controller_theta_to_volt(float theta_target, float theta);

#endif
