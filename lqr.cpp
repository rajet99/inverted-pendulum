#include "lqr.h"

LQR::LQR() {
    // Removed init() since it wasn't defined. Instead, clear the array:
    for (int i = 0; i < STATE_SIZE; ++i) {
        K[i] = 0.0;
    }
}

void LQR::set_K(const double K_gains[4]) {
    for (int i = 0; i < STATE_SIZE; ++i) {
        K[i] = K_gains[i];
    }
}

double LQR::LQR_step(const double current[4], const double target[4]) {

    // Unpack current state
    double x     = current[0]; // Cart Position
    double v     = current[1]; // Cart Velocity
    double theta = current[2]; // Pendulum Angle
    double omega = current[3]; // Pendulum Angular Velocity

    // Unpack target state
    double target_x     = target[0];
    double target_v     = target[1];
    double target_theta = target[2];
    double target_omega = target[3];

    // Error calculation
    double x_error = target_x - x;
    double v_error = target_v - v;
    double theta_error = target_theta - theta;
    double omega_error = target_omega - omega;

    // LQR Controller:
    double control_input = K[0] * x_error + K[1] * v_error + K[2] * theta_error + K[3] * omega_error;
    
    return control_input; 
}