#include "linefollower.h"
#include <sys/time.h>
#include "odometry.h"
#include "motorcontrol.h"
#include <iostream>
#include "subscriber.h"

LineFollower::LineFollower()
{

}

void LineFollower::tick()
{

    if(!foundLine){



    }

}

double  LineFollower::getYaw(double sensorReading)
{
    if(sensorReading>99) return 0;
    return sensorReading*10;

}
