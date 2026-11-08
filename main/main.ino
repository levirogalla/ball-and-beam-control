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

typedef enum {
  APP_STATE_CALCULATE_OVERSHOOT,
  APP_STATE_CALCULATE_STICKTION,
  APP_STATE_SINE_WAVE,
  APP_STATE_SQUARE_WAVE,
  APP_STATE_CHARACTERIZE_BALL_SENSOR,
  APP_STATE_GRAVITY_THING,
  APP_STATE_ESTOP,
} AppState;

static AppState s_app_state;
uint32_t logging_start_us;

// ================== Setup ==================
void setup()
{

  analogReadResolution(14);
  pinMode(A5, OUTPUT); // A5 can be used to measure cycle time using an oscilloscope by connecting the scope to the Arduino Box Motor Leads
  Serial.begin(115200);

  pot_angle_config(MOT_PIN, 3);
  sense_distance_config(BAL_PIN, 3);
  motor_config(PWM_PIN, DIR_PIN);

  delay(300);

  pot_angle_populate();
  motor_init();
  set_control_interval_us(ISR_PERIOD_US); // 2 kHz

  s_app_state = APP_STATE_GRAVITY_THING;
  logging_start_us = micros();
  Serial.println("initialization complete!");
}

// ================== Loop ==================
uint64_t sample = 0;
void loop()
{
  switch (s_app_state)
  {
  case APP_STATE_CALCULATE_OVERSHOOT:
    motor_controller_enable(true);
    calculate_overshoot(1000);
    break;
  case APP_STATE_CALCULATE_STICKTION:
    motor_controller_enable(false);
    calculate_stiction();
    break;
  case APP_STATE_SINE_WAVE:
    motor_controller_enable(true);
    theta_desired_set(0.7 * calculate_sine_wave(8000, 0.0f));
    break;
  case APP_STATE_SQUARE_WAVE:
    motor_controller_enable(true);
    theta_desired_set(0.7 * calculate_square_wave(2000, 0.5, 0.0f));
    break;
  case APP_STATE_CHARACTERIZE_BALL_SENSOR: {
    motor_controller_enable(true);
    float theta =
        0.3 * (0.5 * (calculate_square_wave(13 * 1000, 0.1, 0.0f) + 1) -
               0.5 * (calculate_square_wave(13 * 1000, 0.1, 180.0f) + 1));
    theta_desired_set(theta);
    break;
  }
  case APP_STATE_GRAVITY_THING: {
    motor_controller_enable(true);

    uint32_t elapsed_us = micros() - logging_start_us;
    if (elapsed_us < 5e6) {
      theta_desired_set(0.0);
    } else if (elapsed_us < 15e6) {
      theta_desired_set(0.2);
    } else if (elapsed_us < 25e6) {
      theta_desired_set(-0.2);
    } else if (elapsed_us < 35e6) {
      theta_desired_set(0.3);
    } else if (elapsed_us < 45e6) {
      theta_desired_set(-0.3);
    } else if (elapsed_us < 55e6) {
      theta_desired_set(0.4);
    } else if (elapsed_us < 65e6) {
      theta_desired_set(-0.4);
    } else {
      theta_desired_set(0.0);
    }

    break;
  }
  case APP_STATE_ESTOP:
    motor_controller_enable(false);
    theta_desired_set(0.7 * calculate_square_wave(2000, 0.5, 0.0f));
    Serial.println("Estopped");
    delay(200);
    break;
  }

  float angle = pot_angle_read_eng();
  Serial.print(sample);
  Serial.print(", ");
  Serial.print(micros() - logging_start_us);
  Serial.print(", ");
  Serial.print(theta_desired_get(), 5);
  Serial.print(", ");
  Serial.print(motor_voltage_get(), 5);
  Serial.print(", ");
  Serial.print(angle, 5);
  Serial.print(", ");
  Serial.println(sense_distance_read_eng(), 8);
  Serial.println(pot_angle_read_raw());
}

// ================== Control ISR ==================
static float controller_init_state = 0;
void interval_control_code(void)
{
  sample++;
  pot_angle_populate();
  sense_distance_populate();
  float angle = pot_angle_read_eng();

  if (abs(angle) > 0.8)
  {
    motor_voltage_no_stick_set(0);
    motor_controller_enable(false);
    s_app_state = APP_STATE_ESTOP;
  }

  if (motor_controller_enabled()) {
    float theta_desired = theta_desired_saturated_get();
    motor_voltage_no_stick_set(motor_controller_theta_to_volt_1(
        theta_desired, angle, &controller_init_state));
  }
}
