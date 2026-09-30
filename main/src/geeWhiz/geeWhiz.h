#pragma once
#include <Arduino.h>

// Start a periodic ISR at 'interval_us' using a GPT timer (no PWM conflict)
void set_control_interval_us(uint32_t interval_us);

// Your sketch must define ONE of these names (either is accepted)
extern "C" void interval_control_code(void) __attribute__((weak));
extern "C" void intervalControlCode(void) __attribute__((weak));
