#ifndef PID_H
#define PID_H

#include "vex.h"
#include <cmath> // for fabs

class PID
{
private:
    double kp, ki, kd;
    double previous_error;
    double integral;
    double starti;       // threshold to start integrating
    double integral_max; // max absolute value of integral term

    // Helper to get sign of a number
    int sign(double val)
    {
        return (val > 0) - (val < 0);
    }

public:
    double max_timeout;
    double settle_error;

    PID(double kp_, double ki_, double kd_)
        : kp(kp_),
          ki(ki_),
          kd(kd_),
          previous_error(0.0),
          integral(0.0) {}

    void reset()
    {
        previous_error = 0.0;
        integral = 0.0;
    }

    double update(double error)
    {
        double derivative = error - previous_error;

        // Integrate only if error magnitude is greater than starti (threshold)
        if (fabs(error) <= starti)
        {
            integral += error;
        }

        if ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0))
        {
            integral = 0;
        }
        
        previous_error = error;

        return (error * kp) + (integral * ki) + (derivative * kd);
    }

    // Optional setters
    void setGains(double kp_, double ki_, double kd_)
    {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }

    void setIntegralLimits(double starti_, double integral_max_)
    {
        starti = starti_;
        integral_max = integral_max_;
    }

    void set_constants(double settle_error_, double max_timeout_)
    {
        settle_error = settle_error_;
        max_timeout = max_timeout_;
    }
};

#endif // PID_H
