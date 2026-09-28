#ifndef POSITION_H
#define POSITION_H

void pot_angle_sense_config(int angle_pin);

float pot_angle_sample(int samples);

int pot_angle_read_raw(void);

float pot_angle_read_eng(void);

#endif