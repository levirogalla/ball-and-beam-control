#define ANGLE_SLOPE -0.000359453251f
#define ANGLE_INTERCEPT 1.75789534f

float angle_pot_to_rad(int v) {
    return v * ANGLE_SLOPE + ANGLE_INTERCEPT;
}


