#ifndef VARIBLERADIUSMOTION_H
#define VARIBLERADIUSMOTION_H

#include "trajectoryexecutor.h"
class Odometry;
enum class TurningState{TO_MIN_RADI,TO_HALF_YAW,FROM_HALF_YAW,FROM_MIN_RADI,DONE};

class VaribleRadiusMotion
{

public:
    double tick();


    VaribleRadiusMotion(double startAngle, double endAngle, Odometry* odoFromControl);

private:
    double halfDeltaYaw;
    double yawAtMinRadiReached;
    double yawWhenLeaveMinRadi;
    double endAngle=0;
double previousTime=0;
double angularAccel = 0.17;//rad/s^2 0.17 rad = 10 deg
double angularSpeed=0;
double currentRadius=0;
double speed=0;
double radiusMin = 0.3;
double minAngSpeed= 0.1;// this val is used to see when stright motion is reached, todo adjust value
//double yawTreshold =
Position2D prevPosition;
Odometry* odoFromComtrol;
TurningState turningState;
};

#endif // VARIBLERADIUSMOTION_H
