#include "varibleradiusmotion.h"
#include "trajectoryexecutor.h"
#include "odometry.h"
#include <iostream>

double VaribleRadiusMotion::tick()//return turning radius for motion control, 0 means that stright line motion is reached
{
    double time = TrajectoryExecutor::getSystemTimeSec();
    double dt = time - previousTime;

    Position2D curPosFromControl = odoFromComtrol->pose;// use estimated position from control
    double deltaDist = curPosFromControl.distance(prevPosition);//aprox

    double radiusToApply=radiusMin;

    switch (turningState) {
    case TurningState::FROM_MIN_RADI:{ //state with decreasing radi
        // calc how much we can turn during previous period of time
        angularSpeed += dt*angularAccel;
        double deltaYawToApply = dt*angularSpeed;

        radiusToApply = deltaDist/deltaYawToApply;
        // check if half angle is reached, in such case we should start to increase turning radi, without reaching min radi
        if(std::abs(halfDeltaYaw-curPosFromControl.yaw) > std::abs(halfDeltaYaw-prevPosition.yaw)){
            turningState= TurningState::FROM_HALF_YAW;
        }
        else   if(radiusToApply<radiusMin) {
            radiusToApply = radiusMin;// transition to constatnt radius motion if min radius has been reached
            turningState= TurningState::TO_HALF_YAW;
        }


    }
        break;
    case TurningState::FROM_HALF_YAW:// increase radi
    {
        // calc how much we can turn during previous period of time
        angularSpeed -= dt*angularAccel;
        double deltaYawToApply = dt*angularSpeed;

        radiusToApply = deltaDist/deltaYawToApply;
        // check if trajectory has strightened out enough, platform now sholud be pointing at desired position
        if(std::abs(angularSpeed)<minAngSpeed){
            turningState = TurningState::DONE;
          std::cout<<"turning ended at "<<curPosFromControl.yaw<<" target was "<<endAngle<<std::endl;
            return 0;
        }

    }
        break;

    case TurningState::DONE:{
     return 0;
    }
        break;
    default:
        break;
    }




    prevPosition = curPosFromControl;
    return radiusToApply;
}

VaribleRadiusMotion::VaribleRadiusMotion(double startAngle, double endAngle, Odometry* odoFromControl)
{
    halfDeltaYaw= startAngle+ (endAngle-startAngle)/2;
    this->odoFromComtrol = odoFromControl;
    turningState = TurningState::TO_MIN_RADI;
    this->endAngle = endAngle;
}
