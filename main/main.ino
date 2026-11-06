#include "api/Common.h"
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
  APP_STATE_SINE_WAVE,
  APP_STATE_SQUARE_WAVE,
  APP_STATE_ESTOP,
} AppState;

static AppState s_app_state;
static bool s_enable_control_isr = false;
uint32_t logging_start_us;

// ================== Setup ==================
void setup()
{

  analogReadResolution(14);
  pinMode(A5, OUTPUT); // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.begin(115200);

  pot_angle_config(MOT_PIN, 5);
  motor_config(PWM_PIN, DIR_PIN);

  delay(300);

  pot_angle_populate();
  motor_init();
  set_control_interval_us(ISR_PERIOD_US); // 2 kHz

  s_app_state = APP_STATE_SQUARE_WAVE;
  logging_start_us = micros();
  Serial.println("initialization complete!");
}

// ================== Loop ==================
float motor_voltage = 0;
uint64_t sample = 0;
void loop()
{
  switch (s_app_state)
  {
  case APP_STATE_CALCULATE_OVERSHOOT:
    s_enable_control_isr = true;
    calculate_overshoot(1000);
    break;
  case APP_STATE_CALCULATE_STICKTION:
    s_enable_control_isr = false; // isr would compete for control of the motor
    calculate_stiction();
    break;
  case APP_STATE_SINE_WAVE:
    s_enable_control_isr = true;
    calculate_sine_wave(8000);
    break;
  case APP_STATE_SQUARE_WAVE:
    s_enable_control_isr = true;
    calculate_square_wave(2000, 0.5);
    break;
  case APP_STATE_ESTOP:
    Serial.println("Estopped");
    delay(1000);
    break;
  }

  float angle = pot_angle_read_eng();
  Serial.print(sample);
  Serial.print(", ");
  Serial.print(theta_desired_get(), 5);
  Serial.print(", ");
  Serial.print(motor_voltage, 5);
  Serial.print(", ");
  Serial.println(angle, 5);
  sample++;
}

// ================== Control ISR ==================
static float controller_init_state = 0;
void interval_control_code(void)
{
  pot_angle_populate();
  float angle = pot_angle_read_eng();

  if (abs(angle) > 0.8)
  {
    motor_voltage_no_stick_set(0);
    s_enable_control_isr = false;
    s_app_state = APP_STATE_ESTOP;
  }

  if (s_enable_control_isr)
  {
    float theta_desired = theta_desired_saturated_get();
    motor_voltage = motor_voltage_no_stick_set(motor_controller_theta_to_volt_1(
        theta_desired, angle, &controller_init_state));
  }
}
