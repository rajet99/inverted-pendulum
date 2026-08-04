#include "swing_up.h"
#include <cmath>

double swing_step(const double state[4]) {

    // constants:
    const double m = 0.0295;     // pendulum mass (kg)
    const double g = 9.81;       // gravity (m/s^2)
    const double l = 0.0762;     // 1/2 rod length (m)
    const double I = m * (2.0 * l) * (2.0 * l) / 12.0; // Inertia

    const double r = 50.0; // gain factor
    const double k = 1.0; // centering spring factor
    const double b = 0.5; // spring damping factor

    // unpack current states:
    double x     = state[0];
    double v     = state[1];
    double theta = state[2]; // Assumes radians (0 = down, pi = up)
    double omega = state[3]; // rad/s

    // energy calculations:
    double E_max = m*g*l;
    double KE = 0.5*I*omega*omega;
    double PE = -m*g*l*cos(theta); // theta from bottom/stable equilibrium position
    double E = KE + PE;
    double DE = E - E_max;

    // Åström-Furuta method:
    double sign = (omega * cos(theta)) > 0 ? 1.0 : -1.0;
    double F = r * DE * sign;

    // centering force:
    double s = - k*x - b*v;

    return F + s;
}