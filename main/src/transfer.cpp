#include "transfer.h"

#define ANGLE_SLOPE -3.66300473e-04f
#define ANGLE_INTERCEPT 1.79615437f

#define DISTANCE_SLOPE 1.0875162824513041e-05f
#define DISTANCE_INTERCEPT 0.004743086849837932

#define MOTOR_CONTROLLER_FIRST_ORDER_F0 -1.987525f
#define MOTOR_CONTROLLER_FIRST_ORDER_F1 2.012475f
#define MOTOR_CONTROLLER_FIRST_ORDER_G0 4.8363f
#define MOTOR_CONTROLLER_FIRST_ORDER_G1 -4.9717f

#define BALL_CONTROLLER_PREVIOUS_OUTPUT_COEFF 1.99875f
#define BALL_CONTROLLER_CURRENT_ERROR_COEFF -14.001225f
#define BALL_CONTROLLER_PREVIOUS_ERROR_COEFF 13.998775f

float pot_angle_raw_to_rad(int v) {
  return (float)v * ANGLE_SLOPE + ANGLE_INTERCEPT;
}

float sense_distance_raw_to_m(int v) {
  return (float)v * DISTANCE_SLOPE + DISTANCE_INTERCEPT;
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

float ball_controller_position_to_motor_angle(float position_target,
                                              float position,
                                              float *previous_output,
                                              float *previous_error) {
  float error = position_target - position;
  float output =
      BALL_CONTROLLER_PREVIOUS_OUTPUT_COEFF * (*previous_output) +
      BALL_CONTROLLER_CURRENT_ERROR_COEFF * error +
      BALL_CONTROLLER_PREVIOUS_ERROR_COEFF * (*previous_error);
  *previous_output = output;
  *previous_error = error;
  return output;
}

static bool s_enable_control_isr = false;
void motor_controller_enable(bool enable) { s_enable_control_isr = enable; }
bool motor_controller_enabled() { return s_enable_control_isr; }
