#ifndef OUTER_PID_CONTROLLER_H
#define OUTER_PID_CONTROLLER_H

// Declare the controller function that Simulink will call
double outer_PID_step(double x_pos, double target_pos);

// Declare a reset function to handle initialization
void outer_PID_init(void);

#endif