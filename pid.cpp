#include "pid.h"

PID::PID(double Kp, double Ki, double Kd, double dt)
: Kp_(Kp), Ki_(Ki), Kd_(Kd), dt_(dt), error_integral_(0.0), last_error_(0.0) {}

void PID::reset() {
    error_integral_ = 0.0;
    last_error_ = 0.0;
}

void PID::set_limits(double min, double max) {
    min_input_ = min;
    max_input_ = max;
    limits_ = true;
}


double PID::PID_step(double target, double current) {
    double error = target - current;

    // PID logic:
    double P = Kp_ * error;

    error_integral_ += (error * dt_);
    double I = Ki_ * error_integral_;

    double derivative = (error - last_error_) / dt_;
    double D = Kd_ * derivative;

    last_error_ = error;

    double input = P + I + D;

    // control limiting:
    if (limits_) {
        if (input > max_input_) {
            input = max_input_;
            // Clamp integrator so it doesn't wind up while saturated
            error_integral_ -= (error * dt_); 
        } else if (input < min_input_) {
            input = min_input_;
            // Clamp integrator here as well
            error_integral_ -= (error * dt_);
        }
    }

    return P + I + D;
}