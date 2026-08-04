#ifndef INNER_PID_CONTROLLER_H
#define INNER_PID_CONTROLLER_H

// Declare the controller function that Simulink will call
double inner_PID_step(double theta, double correction_ang);

// Declare a reset function to handle initialization
void inner_PID_init(void);

#endif