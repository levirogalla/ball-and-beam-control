#include "geeWhiz.h"
#include <FspTimer.h>   // UNO R4 timer helper

static FspTimer controlTimer;       // separate GPT for the control ISR

// ---- ISR adapter ----
static void timer_cb_adapter(timer_callback_args_t *) {
  if (interval_control_code) { interval_control_code(); return; }
  if (intervalControlCode)   { intervalControlCode();   return; }
  // neither defined -> do nothing
}

// ---- API ----
void set_control_interval_us(uint32_t interval_us)
{
  if (interval_us == 0)
    interval_us = 1000;
  const float freq_hz = 1000000.0f / interval_us;

  // Use a GPT timer (don’t touch AGT: it’s used by Arduino timebase)
  uint8_t timer_type = GPT_TIMER;
  int8_t  tindex     = FspTimer::get_available_timer(timer_type); // prefer non-PWM GPT

  bool forced = false;
  if (tindex < 0) {
    // As a fallback, allow using a PWM-reserved GPT if needed
    tindex = FspTimer::get_available_timer(timer_type, true);
    forced = (tindex >= 0);
  }
  if (tindex < 0) return;           // no timer available

  if (forced) {
    // Permit using a PWM-reserved GPT for the timer.
    FspTimer::force_use_of_pwm_reserved_timer();
  }

  // Begin periodic timer -> IRQ -> open -> start (open is REQUIRED)
  if (!controlTimer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, freq_hz, 0.0f, timer_cb_adapter)) return;
  if (!controlTimer.setup_overflow_irq()) return;
  if (!controlTimer.open()) return;
  controlTimer.start();
}
