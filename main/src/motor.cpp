#include "geeWhiz/geeWhiz.h"
#include "position.h"
#include "transfer.h"

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
  float initial_theta = pot_angle_read_eng();
  float theta = initial_theta;
  float motor_voltage = 0.0;
  while (abs(theta - initial_theta) < 0.06) {
    setMotorVoltage(motor_voltage);
    delay(200);
    theta = pot_angle_read_eng();

    Serial.print("motor voltage: ");
    Serial.print(motor_voltage, 8);
    Serial.print(" | theta: ");
    Serial.println(theta, 5);

    motor_voltage-=0.001;
  }
  setMotorVoltage(0.0);

  while (true) {}
}

void calculate_overshoot_and_settle(void)
{
  float theta0 = 0;
  float theta1 = 0.2;

  float t0 = millis();
  bool state = 0;
  float theta_desired = theta0;
  while (true)
  {
    if (millis() - t0 > 400)
    {
      state = !state;
      t0 = millis();
    }
    if (state)
    { // high state
      theta_desired = theta1;
    }
    else
    { // low state
      theta_desired = theta0;
    }

    float angle = pot_angle_sample(100);
    // float angle = pot_angle_read_eng();
    set_motor_voltage_no_stick(motor_controller_theta_to_volt(theta_desired, angle));
    // set_motor_voltage_no_stick(theta_desired);

    Serial.print(-0.1);
    Serial.print(", ");
    Serial.print(0.3);
    Serial.print(", ");
    Serial.print(theta_desired);
    Serial.print(", ");
    Serial.println(angle, 5);
  }
}