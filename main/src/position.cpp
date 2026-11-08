#include "position.h"
#include "transfer.h"
#include <analog.h>
#include <atomic>

static int s_angle_pin = 0;
static int s_angle_samples = 1;
static std::atomic<int> s_angle_raw{0};

static int s_position_pin = 0;
static int s_position_samples = 1;
static std::atomic<int> s_position_raw{0};

static void sensor_populate(int pin, int samples, std::atomic<int> *output) {
  int sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }

  // round to nearest integer when averaging
  int raw = (sum + samples / 2) / samples;
  output->store(raw, std::memory_order_relaxed);
}

void pot_angle_config(int angle_pin, int samples) {
  s_angle_pin = angle_pin;
  s_angle_samples = samples > 0 ? samples : 1;
}

void pot_angle_populate(void) {
  sensor_populate(s_angle_pin, s_angle_samples, &s_angle_raw);
}

int pot_angle_read_raw(void) {
  return s_angle_raw.load(std::memory_order_relaxed);
}

float pot_angle_read_eng(void) {
  return pot_angle_raw_to_rad(pot_angle_read_raw());
}

void sense_distance_config(int position_pin, int samples) {
  s_position_pin = position_pin;
  s_position_samples = samples > 0 ? samples : 1;
}

void sense_distance_populate(void) {
  sensor_populate(s_position_pin, s_position_samples, &s_position_raw);
}

int sense_distance_read_raw(void) {
  return s_position_raw.load(std::memory_order_relaxed);
}

float sense_distance_read_eng(void) {
  return sense_distance_raw_to_m(sense_distance_read_raw());
}