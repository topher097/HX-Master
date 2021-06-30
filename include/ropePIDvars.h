#ifndef ROPE_PID_VARS
#define ROPE_PID_VARS

namespace ropePIDvars
{
    //Define Variables we'll be connecting to
    double Setpoint, Input, Output;

    // Define the constants
    double Kp=1, Ki=0.05, Kd=0.25;
}

#endif