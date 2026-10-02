#include "src/geeWhiz/geeWhiz.h"
#include "src/motor.h"
#include "src/position.h"
#include "src/transfer.h"
#include <Arduino.h>
#include <cstdint>

#define ISR_PERIOD_US 500
// ================== Pins ==================
int MOT_PIN = A0; // motor angle sensor
int BAL_PIN = A1; // ball position sensor

int PWM_PIN = D9; // motor PWM
int DIR_PIN = D8; // direction

typedef enum
{
  APP_STATE_CALCULATE_OVERSHOOT,
  APP_STATE_CALCULATE_STICKTION,
} AppState;

static AppState s_app_state;

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
  set_control_interval_us(ISR_PERIOD_US); // 2 kHz

  s_app_state = APP_STATE_CALCULATE_OVERSHOOT;
  Serial.println("initialization complete!");
}

// ================== Loop ==================
uint64_t sample = 0;
void loop()
{
  switch (s_app_state)
  {
  case APP_STATE_CALCULATE_OVERSHOOT:
    calculate_overshoot(1000);
    break;
  case APP_STATE_CALCULATE_STICKTION:
    calculate_stiction();
    break;
  }

  float angle = pot_angle_sample(10);
  // Serial.print(-0.1);
  // Serial.print(", ");
  // Serial.print(0.3);
  // Serial.print(", ");
  Serial.print(sample);
  Serial.print(", ");
  Serial.print(MOTOR_CONTROLLER_KP);
  Serial.print(", ");
  Serial.print(ISR_PERIOD_US);
  Serial.print(", ");
  Serial.print(get_theta_desired(), 5);
  Serial.print(", ");
  Serial.println(angle, 5);
  sample++;
}
// motor voltage: 0.12200007 | theta: -0.13642
// motor voltage: 0.11400005 | theta: -0.21910
// motor voltage: 0.12100007 | theta: -0.28667
// motor voltage: 0.11200005 | theta: -0.35174

// ================== Control ISR ==================
void interval_control_code(void)
{
  float angle = pot_angle_sample(5);
  float theta_desired = get_theta_desired();
  set_motor_voltage_no_stick(motor_controller_theta_to_volt(theta_desired, angle));
}
