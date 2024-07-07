#include "controller_PID/PID.hpp"
#include <iostream>

namespace Controllers 
{
    PID::PID(double kp, double kd, double ki)
    {
        this -> KP = kp;
        this -> KD = kd;
        this -> KI = ki;
    }

    double PID::compute_adjustment(double current_heading)
    {
        double current_error;
        double bias = 360.0;
        double PID_p;
        double PID_d;
        double PID_i;
        double PID_total;

        current_error = abs((heading_goal + bias) - (current_heading + bias)) % 
            360;
        PID_p = KP * current_error;

        PID_total = PID_p;
        last_error = current_error;

        return PID_total;
    }

    void PID::set_heading_goal(double new_heading_goal)
    {
        heading_goal = new_heading_goal; 
        last_error = 0.0;
    }

    double PID::get_error()
    {
        return last_error;
    }
}