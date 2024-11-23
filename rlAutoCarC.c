#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_DISTANCE 30    // Maximum distance from the obstacle
#define MAX_SPEED 60        // Maximum car speed
#define GAMMA 0.4          // Discount factor
#define THRESHOLD 1e-4     // Convergence threshold

// Reward function
double reward(int distance, int speed) {
    if (distance < 3) return -100; // Collision penalty
    if (distance == 3) return 100;
    if (speed > distance) return -10;    // Speeding penalty
    return 10 - speed * 0.5;             // Safe speed reward
}

// Transition function
void next_state(int distance, int speed, const char* action, int* new_distance, int* new_speed) {
    int new_speed_temp = speed;
    if (strcmp(action, "slow_down") == 0) new_speed_temp = fmax(0, speed - 1);
    else if (strcmp(action, "speed_up") == 0) new_speed_temp = fmin(MAX_SPEED, speed + 1);

    *new_distance = fmax(0, distance - new_speed_temp);
    *new_speed = new_speed_temp;
}

// Value iteration
void value_iteration(const char* actions[], int action_count, double V[MAX_DISTANCE + 1][MAX_SPEED + 1]) {
    while (1) {
        double delta = 0.0; // Track max change in value
        double new_V[MAX_DISTANCE + 1][MAX_SPEED + 1] = {0}; // New value table

        for (int distance = 0; distance <= MAX_DISTANCE; ++distance) {
            for (int speed = 0; speed <= MAX_SPEED; ++speed) {
                if (distance == 0) continue; // Skip terminal state

                double max_value = -1e9; // Initialize with a large negative value
                for (int i = 0; i < action_count; ++i) {
                    int next_distance, next_speed;
                    next_state(distance, speed, actions[i], &next_distance, &next_speed);
                    double value = reward(distance, speed) + GAMMA * V[next_distance][next_speed];
                    max_value = fmax(max_value, value);
                }

                new_V[distance][speed] = max_value;
                delta = fmax(delta, fabs(new_V[distance][speed] - V[distance][speed]));
            }
        }

        memcpy(V, new_V, sizeof(new_V));
        if (delta < THRESHOLD) break; // Stop if converged
    }
}

int main() {
    const char* actions[] = {"slow_down", "maintain_speed", "speed_up"};
    double V[MAX_DISTANCE + 1][MAX_SPEED + 1] = {0}; // Value table
    value_iteration(actions, 3, V);

    // Print results
    int res = -9999;
    int finalSpeed = -9999;
    int finalDistance = -9999;
    int distance;
    printf("Enter your current speed: ");
    scanf("%d", &distance);
    for (int speed = 0; speed <= MAX_SPEED; ++speed) {
        if (res < V[distance][speed]) {
            res = V[distance][speed];
            finalDistance = distance;
            finalSpeed = speed;
        }
    }
    printf("final: %d and speed: %d\n", finalDistance, finalSpeed);

    return 0;
}

