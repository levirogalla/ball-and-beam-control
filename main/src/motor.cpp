#include "Arduino.h"
#include "geeWhiz/geeWhiz.h"
#include "position.h"
#include "transfer.h"
#include <atomic>
#include <cmath>
#include <pwm.h>

#define POS_V_STICTION -0.1034f
#define NEG_V_STICTION -0.5763f
#define THETA_DESIRED_MAX 0.7f
#define THETA_SWEEP_AMPLITUDE 1

static int s_motor_dir_pin;
static bool s_motor_dir_pin_configured = false;
static PwmOut s_motor_pwm(0);
static bool s_motor_pwm_configured = false;
static constexpr float PWM_HZ = 24000.0f;
static constexpr float VMAX = 6.0f;
static constexpr float DUTY_MAX = 99.2f;
static std::atomic<float> s_theta_desired{0.0f};
static float s_motor_last_set_voltage = 0;

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

void motor_voltage_no_stick_set(float volts) {
  if (volts > 0.0f)
  {
    s_motor_last_set_voltage = volts + POS_V_STICTION;
  }
  else if (volts < 0.0f)
  {
    s_motor_last_set_voltage = volts + NEG_V_STICTION;
  }
  else
  {
    s_motor_last_set_voltage = 0;
  }
  motor_voltage_set(s_motor_last_set_voltage);
}

float motor_voltage_get() { return s_motor_last_set_voltage; }

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

float calculate_sine_wave(uint32_t period_ms) {
  if (period_ms == 0)
  {
    return 0.0f;
  }

  float cycle_fraction = static_cast<float>(millis() % period_ms) /
                         static_cast<float>(period_ms);
  float phase = 2.0f * PI * cycle_fraction;
  return THETA_SWEEP_AMPLITUDE * sinf(phase);
}

float calculate_square_wave(uint32_t period_ms, float duty_cycle) {
  if (period_ms == 0)
  {
    return 0.0f;
  }

  float cycle_fraction = static_cast<float>(millis() % period_ms) /
                         static_cast<float>(period_ms);
  return cycle_fraction < duty_cycle ? THETA_SWEEP_AMPLITUDE
                                     : -THETA_SWEEP_AMPLITUDE;
}

/// @brief Calculate and print stiction of motor, disables interrupts
void calculate_stiction(void)
{
  const float voltage_step = 0.001;
  const float theta_threshold = 0.02;

  float theta;
  float motor_voltage = 0.0;

  // Bring motor to theta = 0 using the normal controller.
  Serial.println("Setting theta to 0 using closed loop control...");
  theta_desired_set(0.0);
  motor_controller_enable(true);
  delay(2000);
  motor_controller_enable(false);

  // Find stiction in both directions.
  float stiction_vals[2] = {0, 0};

  for (int direction = -1; direction <= 1; direction += 2) {
    motor_voltage = motor_voltage_get();
    bool moved = false;
    float theta_init = pot_angle_read_eng();

    while (!moved) {
      for (int t = 30; t > 0; t--) {
        theta = pot_angle_read_eng();

        motor_voltage_set(motor_voltage);

        if (abs(theta - theta_init) >= theta_threshold) {
          moved = true;
          break;
        }

        Serial.print(motor_voltage, 8);
        Serial.print(",");
        Serial.println(theta, 5);

        delay(4);
      }

      motor_voltage += direction * voltage_step;
    }

    Serial.println("done");

    stiction_vals[(direction + 1) / 2] = motor_voltage;

    // Return to zero using the normal controller.
    motor_controller_enable(true);
    delay(2000);
    motor_controller_enable(false);
  }

  motor_controller_enable(true);
  theta_desired_set(0.0);

  Serial.print("negative voltage stiction: ");
  Serial.println(stiction_vals[0], 8);
  Serial.print("positive voltage stiction: ");
  Serial.println(stiction_vals[1], 8);

  Serial.println("stiction calculation complete... delaying for 10 seconds");
  delay(1000 * 60 * 10);

  motor_controller_enable(false);
}