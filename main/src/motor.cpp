#include "geeWhiz/geeWhiz.h"

#define POS_V_STICTION 0.120f
#define NEG_V_STICTION -0.101f

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
void calculate_stiction(void) {
  float initial_theta = angle_pot_to_rad(analogRead(MOT_PIN));
  float theta = initial_theta;
  float motor_voltage = 0.0;
  while (abs(theta - initial_theta) < 0.06) {
    setMotorVoltage(motor_voltage);
    delay(200);
    theta = angle_pot_to_rad(analogRead(MOT_PIN));

    Serial.print("motor voltage: ");
    Serial.print(motor_voltage, 8);
    Serial.print(" | theta: ");
    Serial.println(theta, 5);

    motor_voltage-=0.001;
  }
  setMotorVoltage(0.0);

  while (true) {}
}