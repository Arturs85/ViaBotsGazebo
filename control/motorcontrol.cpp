#include "motorcontrol.h"
#include "subscriber.h"
#include "odometry.h"

MotorControl::MotorControl(double wheelBase, double wheelRadius)
{
    this->wheelBase = wheelBase;
    this->wheelRadius = wheelRadius;
    odometryFromControl = new Odometry();

}

MotorControl::MotorControl(double wheelBase, double wheelRadius, Subscriber* subscriber)
{
    this->wheelBase = wheelBase;
    this->wheelRadius = wheelRadius;
    this->subscriber = subscriber;
    odometryFromControl = new Odometry();
}

void MotorControl::setSpeed(double speed, double radius)
{
  //  std::cout<<"setSpeed motorcontrol\n";
    //set given speed for outer wheel, calculate for other
    double wb05 = wheelBase/2;
    if(radius<0){
        leftWheelSpeed=speed/wheelRadius; // converting from m/s to rad/s
        rightWheelSpeed = leftWheelSpeed*(radius+wb05)/(radius-wb05);
    }else
        if(radius>0){
            rightWheelSpeed=speed/wheelRadius;
            leftWheelSpeed = rightWheelSpeed*(radius-wb05)/(radius+wb05);

        }else{//todo - how to pass direction when turning about center
            rightWheelSpeed =0;
            leftWheelSpeed = 0;
        }
    sendWheelSpeeds();
    odometryFromControl->updateAnglesFromSpeed(leftWheelSpeed,rightWheelSpeed);

}




void MotorControl::calcWheelSpeeds()
{

}

void MotorControl::sendWheelSpeeds()
{
    Subscriber::sendWheelSpeeds(leftWheelSpeed,rightWheelSpeed);
}
