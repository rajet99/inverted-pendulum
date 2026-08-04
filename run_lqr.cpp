#include "lqr.h"

double lqr_step(const double gains[4], const double current[4], const double target[4]) {
    static LQR lqr;
    lqr.set_K(gains);
    double input = lqr.LQR_step(current, target);

    return input;
}
