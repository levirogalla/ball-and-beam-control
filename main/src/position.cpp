#include "transfer.h"
#include "position.h"
#include <analog.h>

static int s_angle_pin;

void pot_angle_sense_config(int angle_pin) {
  s_angle_pin = angle_pin;
}

float pot_angle_sample(int n)
{
  float sum = 0;
  for (int i = 0; i < n; i++)
  {
    sum += pot_angle_read_eng();
  }
  return sum / (float)n;
}

int pot_angle_read_raw(void)
{
  return analogRead(s_angle_pin);
}

float pot_angle_read_eng(void)
{
  return pot_angle_raw_to_rad(pot_angle_read_raw());
}