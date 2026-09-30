#include "geeWhiz/geeWhiz.h"
#include "position.h"
#include "transfer.h"
#include <pwm.h>

#define POS_V_STICTION 0.120f
#define NEG_V_STICTION -0.101f

static PwmOut *s_motor_pwm;
static constexpr float PWM_HZ = 24000.0f;
static int s_motor_dir_pin;

// must be called before motor init
void motor_config(int pwm_pin, int dir_pin)
{
  PwmOut _motor_pwm(pwm_pin);
  *s_motor_pwm = _motor_pwm;
  s_motor_dir_pin = dir_pin;
}

int motor_init()
{
  pinMode(s_motor_dir_pin, OUTPUT);
  digitalWrite(s_motor_dir_pin, LOW);

  // Start 24 kHz PWM on D9 at 0% duty
  if (s_motor_pwm == nullptr)
  {
    return -1;
  }
  s_motor_pwm->begin(PWM_HZ, 0.0f);
  return 0;
}

void set_motor_voltage_no_stick(float volts)
{
  if (volts > 0.0f)
  {
    setMotorVoltage(volts + POS_V_STICTION);
  }
  else if (volts < 0.0f)
  {
    setMotorVoltage(volts + NEG_V_STICTION);
  }
  else
  {
    setMotorVoltage(0);
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
    setMotorVoltage(motor_voltage);
    delay(200);
    theta = pot_angle_read_eng();

    Serial.print("motor voltage: ");
    Serial.print(motor_voltage, 8);
    Serial.print(" | theta: ");
    Serial.println(theta, 5);

    motor_voltage -= 0.001;
  }
  setMotorVoltage(0.0);

  while (true)
  {
  }
}
