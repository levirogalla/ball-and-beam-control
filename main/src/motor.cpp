#include "geeWhiz/geeWhiz.h"
#include "position.h"
#include "transfer.h"
#include <atomic>
#include <cmath>
#include <pwm.h>

#define POS_V_STICTION 0.120f
#define NEG_V_STICTION -0.101f
#define THETA_DESIRED_MAX 0.7f
#define THETA_SWEEP_AMPLITUDE 2.0f

static int s_motor_dir_pin;
static bool s_motor_dir_pin_configured = false;
static PwmOut s_motor_pwm(0);
static bool s_motor_pwm_configured = false;
static constexpr float PWM_HZ = 24000.0f;
static constexpr float VMAX = 6.0f;
static constexpr float DUTY_MAX = 99.2f;
static std::atomic<float> s_theta_desired{0.0f};

// must be called before motor init
void motor_config(int pwm_pin, int dir_pin)
{
  PwmOut configured_pwm(pwm_pin);
  s_motor_pwm = configured_pwm;
  s_motor_pwm_configured = true;

  s_motor_dir_pin = dir_pin;
  s_motor_dir_pin_configured = true;
}

int motor_init()
{
  if (!s_motor_pwm_configured || !s_motor_dir_pin_configured)
  {
    return -1;
  }

  pinMode(s_motor_dir_pin, OUTPUT);
  digitalWrite(s_motor_dir_pin, LOW);

  s_motor_pwm.begin(PWM_HZ, 0.0f);
  return 0;
}

static void motor_voltage_set(float volts)
{
  if (volts > VMAX)
    volts = VMAX;
  if (volts < -VMAX)
    volts = -VMAX;

  digitalWrite(s_motor_dir_pin, (volts >= 0.0f) ? HIGH : LOW);
  float duty = (fabsf(volts) / VMAX) * 100.0f;
  if (duty > DUTY_MAX)
    duty = DUTY_MAX;

  s_motor_pwm.pulse_perc(duty);
}

void motor_voltage_no_stick_set(float volts)
{
  if (volts > 0.0f)
  {
    motor_voltage_set(volts + POS_V_STICTION);
  }
  else if (volts < 0.0f)
  {
    motor_voltage_set(volts + NEG_V_STICTION);
  }
  else
  {
    motor_voltage_set(0);
  }
}

static float theta_desired_saturate(float theta_desired)
{
  if (theta_desired > THETA_DESIRED_MAX)
  {
    return THETA_DESIRED_MAX;
  }
  else if (theta_desired < -THETA_DESIRED_MAX)
  {
    return -THETA_DESIRED_MAX;
  }
  else
  {
    return theta_desired;
  }
}

void theta_desired_set(float theta)
{
  s_theta_desired.store(theta, std::memory_order_relaxed);
}

float theta_desired_get(void)
{
  return s_theta_desired.load(std::memory_order_relaxed);
}

float theta_desired_saturated_get(void)
{
  return theta_desired_saturate(theta_desired_get());
}

void calculate_overshoot(uint32_t period_ms)
{
  float theta0 = 0.0f;
  float theta1 = 0.2f;

  static uint32_t t0 = millis();
  static bool state = false;

  if (millis() - t0 > period_ms)
  {
    state = !state;
    t0 = millis();
  }

  float theta_target = state ? theta1 : theta0;
  theta_desired_set(theta_target);
}

void calculate_sine_wave(uint32_t period_ms)
{
  if (period_ms == 0)
  {
    theta_desired_set(0.0f);
    return;
  }

  float cycle_fraction = static_cast<float>(millis() % period_ms) /
                         static_cast<float>(period_ms);
  float phase = 2.0f * PI * cycle_fraction;
  theta_desired_set(THETA_SWEEP_AMPLITUDE * sinf(phase));
}

/// @brief Calculate and print stiction of motor, disables interrupts
void calculate_stiction(void) {
  pot_angle_populate();
  float initial_theta = pot_angle_read_eng();
  float theta = initial_theta;
  float motor_voltage = 0.0;
  bool done = false;
  while (!done) {
    motor_voltage_set(motor_voltage);

    for (int t = 50; t > 0; t--) {
      pot_angle_populate();
      theta = pot_angle_read_eng();
      delay(4);

      if (abs(theta - initial_theta) >= 0.06) {
        done = true;
        break;
      }

      Serial.print(motor_voltage, 8);
      Serial.print(",");
      Serial.println(theta, 5);
    }
    motor_voltage -= 0.001;
  }
  motor_voltage_set(0.0);
  Serial.println("stiction calculation complete... delaying for 10 seconds");
  delay(1000 * 10);
}
