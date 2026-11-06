#include "transfer.h"

#define ANGLE_SLOPE -0.000359453251f
#define ANGLE_INTERCEPT 1.75789534f

#define MOTOR_CONTROLLER_FIRST_ORDER_F0 -1.987525f
#define MOTOR_CONTROLLER_FIRST_ORDER_F1 2.012475f
#define MOTOR_CONTROLLER_FIRST_ORDER_G0 4.8363f
#define MOTOR_CONTROLLER_FIRST_ORDER_G1 -4.9717f

float pot_angle_raw_to_rad(int v) {
  return (float)v * ANGLE_SLOPE + ANGLE_INTERCEPT;
}

float motor_controller_theta_to_volt_0(float theta_target, float theta) {
  float err = theta_target - theta;
  return err * MOTOR_CONTROLLER_KP;
}

float motor_controller_theta_to_volt_1(float theta_target, float theta,
                                       float *previous_state) {
  float err = theta_target - theta;
  float state = err - (MOTOR_CONTROLLER_FIRST_ORDER_F0 /
                       MOTOR_CONTROLLER_FIRST_ORDER_F1) *
                          *previous_state;
  float u =
      (MOTOR_CONTROLLER_FIRST_ORDER_G1 / MOTOR_CONTROLLER_FIRST_ORDER_F1) *
          state +
      (MOTOR_CONTROLLER_FIRST_ORDER_G0 / MOTOR_CONTROLLER_FIRST_ORDER_F1) *
          *previous_state;
  *previous_state = state;
  return u;
}

static bool s_enable_control_isr = false;
void motor_controller_enable(bool enable) { s_enable_control_isr = enable; }
bool motor_controller_enabled() { return s_enable_control_isr; }
