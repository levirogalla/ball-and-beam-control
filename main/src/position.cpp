#include "position.h"
#include "transfer.h"
#include <analog.h>
#include <atomic>

static int s_angle_pin = 0;
static int s_angle_samples = 1;
static std::atomic<int> s_angle_raw{0};

void pot_angle_config(int angle_pin, int samples) {
  s_angle_pin = angle_pin;
  s_angle_samples = samples > 0 ? samples : 1;
}

void pot_angle_populate(void) {
  int sum = 0;
  for (int i = 0; i < s_angle_samples; i++) {
    sum += analogRead(s_angle_pin);
  }

  int raw = (sum + s_angle_samples / 2) / s_angle_samples;
  s_angle_raw.store(raw, std::memory_order_relaxed);
}

int pot_angle_read_raw(void)
{
  return s_angle_raw.load(std::memory_order_relaxed);
}

float pot_angle_read_eng(void)
{
  return pot_angle_raw_to_rad(pot_angle_read_raw());
}