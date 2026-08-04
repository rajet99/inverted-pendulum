#ifndef LQR_H
#define LQR_H

class LQR {
public:
    LQR();

    void set_K(const double K_gains[4]); // Fixed signature to match .cpp

    double LQR_step(const double current[4], const double target[4]);

private:
    static const int STATE_SIZE = 4;
    double K[STATE_SIZE];
};

#endif