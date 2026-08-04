#include "inner_pid.h"
#define pi 3.14159265358979323846

static double error_integral = 0.0;
static double last_error = 0.0;

void inner_PID_init(void) {
    error_integral = 0.0;
    last_error = 0.0;
}

double inner_PID_step(double theta, double correction_ang) {
    double target_angle = pi + correction_ang;

    // Gains:
    double Kp = 10.0;
    double Ki = 0.5;//11.0;
    double Kd = 0.0;//0.0057;

    double dt = 0.001; // 1 kHz timestep
    double I_max = 4; // current limit (A) -> drift

    // ------PID controller:--------
    double error = target_angle - theta;
    double P = Kp * error;

    error_integral += (error * dt);
    double I = Ki * error_integral;

    double derivative = (error - last_error) / dt;
    double D = Kd * derivative;

    double I_out = P + I + D;

    // ---- current limiting ------
    if (I_out > I_max) {
        I_out = I_max;
        error_integral -= (error * dt); // Anti-windup
    } else if (I_out < -I_max) {
        I_out = -I_max;
        error_integral -= (error * dt);
    }

    return I_out;
}