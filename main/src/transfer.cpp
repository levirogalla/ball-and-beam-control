#define ANGLE_SLOPE -0.000359453251f
#define ANGLE_INTERCEPT 1.75789534f

float pot_angle_raw_to_rad(int v)
{
    return (float)v * ANGLE_SLOPE + ANGLE_INTERCEPT;
}

#define MOTOR_CONTROLLER_KP -100

float motor_controller_theta_to_volt(float theta_target, float theta)
{
    float err = theta_target - theta;
    return err * MOTOR_CONTROLLER_KP;
}
