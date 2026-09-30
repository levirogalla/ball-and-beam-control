#include "geeWhiz/geeWhiz.h"
#include "position.h"
#include "transfer.h"
#include <pwm.h>

#define POS_V_STICTION 0.120f
#define NEG_V_STICTION -0.101f

static int s_motor_dir_pin;
static bool s_motor_dir_pin_configured = false;
static PwmOut s_motor_pwm(0);
static bool s_motor_pwm_configured = false;
static constexpr float PWM_HZ = 24000.0f;
static constexpr float VMAX = 6.0f;
static constexpr float DUTY_MAX = 99.2f;

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

static void set_motor_voltage(float volts)
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

void set_motor_voltage_no_stick(float volts)
{
  if (volts > 0.0f)
  {
    set_motor_voltage(volts + POS_V_STICTION);
  }
  else if (volts < 0.0f)
  {
    set_motor_voltage(volts + NEG_V_STICTION);
  }
  else
  {
    set_motor_voltage(0);
  }
}

/// @brief Calculate and print stiction of motor
void calculate_stiction(void)
{
  float initial_theta = pot_angle_read_eng();
  float theta = initial_theta;
  float motor_voltage = 0.0;
  while (abs(theta - initial_theta) < 0.06)
  {
    set_motor_voltage(motor_voltage);
    delay(200);
    theta = pot_angle_read_eng();

    Serial.print("motor voltage: ");
    Serial.print(motor_voltage, 8);
    Serial.print(" | theta: ");
    Serial.println(theta, 5);

    motor_voltage -= 0.001;
  }
  set_motor_voltage(0.0);

  while (true)
  {
  }
}
