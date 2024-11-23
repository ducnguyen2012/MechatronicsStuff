#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace std;

// Constants
const int MAX_DISTANCE = 30;    // Maximum distance from the obstacle
const int MAX_SPEED = 60;        // Maximum car speed
const double GAMMA = 0.4;       // Discount factor
const double THRESHOLD = 1e-4;  // Convergence threshold

// Reward function
double reward(int distance, int speed) {
    if (distance < 3) return -100; // Collision penalty
    if (distance == 3) return 100;
    if (speed > distance) return -10;    // Speeding penalty
    return 10 - speed * 0.5;             // Safe speed reward
}

// Transition function
pair<int, int> next_state(int distance, int speed, const string& action) {
    int new_speed = speed;
    if (action == "slow_down") new_speed = max(0, speed - 1);
    else if (action == "speed_up") new_speed = min(MAX_SPEED, speed + 1);

    int new_distance = max(0, distance - new_speed);
    return {new_distance, new_speed};
}

// Value iteration
vector<vector<double>> value_iteration(const vector<string>& actions) {
    vector<vector<double>> V(MAX_DISTANCE + 1, vector<double>(MAX_SPEED + 1, 0.0)); // Value table

    while (true) {
        double delta = 0.0; // Track max change in value
        auto new_V = V;

        for (int distance = 0; distance <= MAX_DISTANCE; ++distance) {
            for (int speed = 0; speed <= MAX_SPEED; ++speed) {
                if (distance == 0) continue; // Skip terminal state

                double max_value = -1e9; // Initialize with a large negative value
                for (const string& action : actions) {
                    auto [next_distance, next_speed] = next_state(distance, speed, action);
                    double value = reward(distance, speed) + GAMMA * V[next_distance][next_speed];
                    max_value = max(max_value, value);
                }

                new_V[distance][speed] = max_value;
                delta = max(delta, fabs(new_V[distance][speed] - V[distance][speed]));
            }
        }

        V = new_V;
        if (delta < THRESHOLD) break; // Stop if converged
    }

    return V;
}

int main() {
    vector<string> actions = {"slow_down", "maintain_speed", "speed_up"};
    auto V = value_iteration(actions);

    // Print results
    int res = -9999;
    int finalSpeed = -9999;
    int finalDistance = -9999;
    //for (int distance = 0; distance <= MAX_DISTANCE; ++distance) {
    int distance;
    cout<<"Enter your current speed: ";cin>>distance;
        for (int speed = 0; speed <= MAX_SPEED; ++speed) {
            if (res < V[distance][speed]) {
                res = V[distance][speed];
                finalDistance = distance;
                finalSpeed = speed;
            }
            //cout << "V[Distance " << distance << ", Speed " << speed << "] = " << V[distance][speed] << "\n";
            
        }
        cout<<"final: "<<finalDistance<<" and speed: "<<finalSpeed<<"\n";
            
    //}

    return 0;
}

