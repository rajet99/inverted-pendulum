#include "outer_pid.h"

static double error_integral = 0.0;
static double last_error = 0.0;

void outer_PID_init(void) {
    error_integral = 0.0;
    last_error = 0.0;
}

double outer_PID_step(double x_pos, double target_pos) {
    // Gains:
    double Kp = -0.0001;
    double Ki = -0.001;//-0.001;
    double Kd = -0.00001;

    double dt = 0.001; // 1 kHz timestep

    // ------PID controller:--------
    double error = target_pos  - x_pos;
    double P = Kp * error;

    error_integral += (error * dt);
    double I = Ki * error_integral;

    double derivative = (error - last_error) / dt;
    double D = Kd * derivative;

    double theta_opt = P + I + D;

    return theta_opt;
}