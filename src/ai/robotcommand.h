#ifndef ROBOTCOMMAND_H
#define ROBOTCOMMAND_H

#include "position.h"

struct RobotCommand
{
    Position fin_pos;
    Position fin_vel;

    char angleMode;
    double maxSpeed;

    float kickspeedx;
    float kickspeedz;

    RobotCommand()
    {
        angleMode=0;
        maxSpeed=0;
        kickspeedx=0;
        kickspeedz=0;
    }
};


#endif // ROBOTCOMMAND_H
