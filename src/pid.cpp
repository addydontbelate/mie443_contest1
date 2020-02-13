#include <iostream>
#include <cmath>
#include "pid.h"

PID::PID()
{
    dt = 0.0;
    max = 0.0;
    min = 0.0;
    Kp = 0.0;
    Kd = 0.0;
    Ki = 0.0;
    prev_error = 0.0;
    integral = 0.0;
}

PID::PID(float i_dt, float i_max, float i_min, float i_Kp, float i_Kd, float i_Ki)
{
    dt = i_dt;
    max = i_max;
    min = i_min;
    Kp = i_Kp;
    Kd = i_Kd;
    Ki = i_Ki;
    prev_error = 0.0;
    integral = 0.0;
}

float PID::calculate(float setpoint, float pv)
{
    // calculate error
    float error = setpoint - pv;

    // proportional term
    float p_out = Kp*error;

    // integral term
    integral += error*dt;
    float i_out = Ki*integral;

    // derivative term
    float derivative = (error - prev_error) / dt;
    float d_out = Kd * derivative;

    // calculate total output
    float output = p_out + i_out + d_out;

    // restrict to max/min
    if (output > max)
        output = max;
    else if (output < min)
        output = min;

    // save error to previous error
    prev_error = error;

    return output;
}