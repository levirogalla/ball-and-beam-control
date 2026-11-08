#ifndef POSITION_H
#define POSITION_H

void pot_angle_config(int angle_pin, int samples);

void pot_angle_populate(void);

int pot_angle_read_raw(void);

float pot_angle_read_eng(void);

void sense_distance_config(int distance_pin, int samples);

void sense_distance_populate(void);

int sense_distance_read_raw(void);

float sense_distance_read_eng(void);

#endif