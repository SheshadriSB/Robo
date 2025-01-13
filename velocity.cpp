#include <iostream>
#include <cmath> // For sqrt()

using namespace std;

// Function to calculate and display wheel velocities
void wheelVelocity(const float commandVelocities[3], float L0) {
    const float SQRT_3_OVER_2 = sqrt(3) / 2.0; // √3/2
    const float HALF = 0.5;

    // Extract command velocities
    float Vx = commandVelocities[0];
    float Vy = commandVelocities[1];
    float omega = commandVelocities[2];

    // Calculate wheel velocities
    float Va = (-HALF * Vx) + (SQRT_3_OVER_2 * Vy) + (L0 * omega);
    float Vb = (-HALF * Vx) - (SQRT_3_OVER_2 * Vy) + (L0 * omega);
    float Vc = Vx + (L0 * omega);

    // Display results
    cout << "\nWheel Velocities:" << endl;
    cout << "Va = " << Va << endl;
    cout << "Vb = " << Vb << endl;
    cout << "Vc = " << Vc << endl;
}

int main() {
    // Command velocities: [Vx, Vy, ω]
    float commandVelocities[3] = {1.5, -0.8, 0.4}; // Example random values for Vx, Vy, ω
    float L0 = 1.0; // Example wheel offset value

    // Call the wheelVelocity function
    wheelVelocity(commandVelocities, L0);

    return 0;
}
