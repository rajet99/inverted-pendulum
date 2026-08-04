#ifndef RUN_LQR_H
#define RUN_LQR_H

double lqr_step(const double gains[4], const double current[4], const double target[4]);

#endif