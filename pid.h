#ifndef PID_H
#define PID_H

class PID{
    public:
        // constructor:
        PID(double Kp = 0.0, double Ki = 0.0, double Kd = 0.0, double dt = 0.001);

        void reset();
        void set_limits(double min, double max);

        double PID_step(double target, double current);

    private:
        // parameters:
        double Kp_;
        double Ki_;
        double Kd_;
        double dt_;

        // limits:
        bool limits_;
        double min_input_;
        double max_input_;

        // controller state memory
        double error_integral_;
        double last_error_;
};

    


#endif 