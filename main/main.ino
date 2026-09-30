#include <Arduino.h>
#include "src/geeWhiz/geeWhiz.h"
#include "src/transfer.h"
#include "src/position.h"
#include "src/motor.h"

// ================== Pins ==================
int MOT_PIN = A0; // motor angle sensor
int BAL_PIN = A1; // ball position sensor

int PWM_PIN = D9; // motor PWM
int DIR_PIN = D8; // direction

static volatile float s_theta_desired = 0.0f;

void calculate_stiction(void);

// ================== Setup ==================
void setup()
{

  analogReadResolution(14);
  pinMode(A5, OUTPUT); // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.begin(115200);

  pot_angle_sense_config(MOT_PIN);
  motor_config(PWM_PIN, DIR_PIN);

  delay(300);

  motor_init();

  set_control_interval_us(100); // 10 kHz
  // setMotorVoltage(0.0f);
  // calculate_stiction();

  Serial.println("geeWhiz Started");
}

// ================== Loop ==================
void loop()
{
  float theta0 = 0.0f;
  float theta1 = 0.2f;

  static uint32_t t0 = millis();
  static bool state = false;

  if (millis() - t0 > 400)
  {
    state = !state;
    t0 = millis();
  }

  float theta_desired = state ? theta1 : theta0;
  noInterrupts();
  s_theta_desired = theta_desired;
  interrupts();

  float angle = pot_angle_read_eng();

  Serial.print(-0.1);
  Serial.print(", ");
  Serial.print(0.3);
  Serial.print(", ");
  Serial.print(theta_desired, 5);
  Serial.print(", ");
  Serial.println(angle, 5);
}
// motor voltage: 0.12200007 | theta: -0.13642
// motor voltage: 0.11400005 | theta: -0.21910
// motor voltage: 0.12100007 | theta: -0.28667
// motor voltage: 0.11200005 | theta: -0.35174

// ================== Control ISR ==================
void interval_control_code(void)
{
  float angle = pot_angle_read_eng();
  set_motor_voltage_no_stick(motor_controller_theta_to_volt(s_theta_desired, angle));
}
