#define SQRT_3_OVER_2 0.86602540378
#define HALF 0.5
void wheelVelocity(const float commandVelocities[3], float L0) {
    float Vx = commandVelocities[0];
    float Vy = commandVelocities[1];
    float omega = commandVelocities[2];
    float Va = (-HALF * Vx) + (SQRT_3_OVER_2 * Vy) + (L0 * omega);
    float Vb = (-HALF * Vx) - (SQRT_3_OVER_2 * Vy) + (L0 * omega);
    float Vc = Vx + (L0 * omega);
}

