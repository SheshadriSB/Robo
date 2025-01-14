#include "raylib.h"
#include <cmath>

const int screenWidth = 800;
const int screenHeight = 600;

// Robot parameters
struct Robot {
    Vector2 position;
    float orientation;
    float wheelBase;  // Distance between wheels
    float wheelRadius;  // Radius of wheels
    float wheelSpeed1, wheelSpeed2, wheelSpeed3;
};

// Function to calculate wheel speeds using inverse kinematics
void CalculateWheelSpeeds(Robot &robot, float linearVelocity, float angularVelocity) {
    float l = robot.wheelBase;

    robot.wheelSpeed1 = (linearVelocity + l * angularVelocity) / 2;
    robot.wheelSpeed2 = (linearVelocity - l * angularVelocity) / 2;
    robot.wheelSpeed3 = angularVelocity;
}

// Update robot state
void UpdateRobot(Robot &robot, float deltaTime) {
    float linearVelocity = 0.0f;
    float angularVelocity = 0.0f;

    // Keyboard inputs
    if (IsKeyDown(KEY_W)) linearVelocity += 1.0f;
    if (IsKeyDown(KEY_S)) linearVelocity -= 1.0f;
    if (IsKeyDown(KEY_A)) angularVelocity += 1.0f;
    if (IsKeyDown(KEY_D)) angularVelocity -= 1.0f;

    // Calculate wheel speeds
    CalculateWheelSpeeds(robot, linearVelocity, angularVelocity);

    // Update robot state
    robot.orientation += robot.wheelSpeed3 * deltaTime;
    robot.position.x += cos(robot.orientation) * robot.wheelSpeed1 * deltaTime;
    robot.position.y += sin(robot.orientation) * robot.wheelSpeed1 * deltaTime;
}

int main() {
    InitWindow(screenWidth, screenHeight, "3-Wheel Omni Robot Simulation with Inverse Kinematics");
    SetTargetFPS(60);

    Robot robot = {{screenWidth / 2.0f, screenHeight / 2.0f}, 0.0f, 1.0f, 0.2f, 0.0f, 0.0f, 0.0f};

    while (!WindowShouldClose()) {
        float deltaTime = GetFrameTime();

        UpdateRobot(robot, deltaTime);

        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Draw the robot
        DrawCircleV(robot.position, robot.wheelRadius, BLUE);

        // Drawing wheels
        DrawCircle(robot.position.x + cos(robot.orientation) * robot.wheelBase,
                   robot.position.y + sin(robot.orientation) * robot.wheelBase, 
                   robot.wheelRadius, GREEN);
        DrawCircle(robot.position.x - cos(robot.orientation) * robot.wheelBase, 
                   robot.position.y - sin(robot.orientation) * robot.wheelBase, 
                   robot.wheelRadius, GREEN);
        DrawCircle(robot.position.x, robot.position.y, 
                   robot.wheelRadius, RED);

        DrawLine(robot.position.x, robot.position.y, 
                 robot.position.x + cos(robot.orientation) * robot.wheelBase, 
                 robot.position.y + sin(robot.orientation) * robot.wheelBase, RED);

        DrawText("Use W/S for linear movement and A/D for rotation", 10, 10, 20, DARKGRAY);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
