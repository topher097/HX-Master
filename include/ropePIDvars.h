#ifndef ROPE_PID_VARS
#define ROPE_PID_VARS

namespace ropePIDvars
{
    //Define Variables we'll be connecting to
    double Setpoint, Input, Output;

    //Define the aggressive and conservative Tuning Parameters
    double aggKp=4, aggKi=0.2, aggKd=1;
    double consKp=1, consKi=0.05, consKd=0.25;
}

#endif