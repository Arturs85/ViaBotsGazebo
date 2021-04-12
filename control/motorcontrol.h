#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

class Subscriber;
class Odometry;

class MotorControl
{
public:
    MotorControl (double wheelBase, double wheelRadius);
    MotorControl (double wheelBase, double wheelRadius, Subscriber* subscriber);

    void setSpeed(double speed, double radius);
double getRightWheelSpeed();//for consumers like topic advertisers
double getLefttWheelSpeed();

void setTargetPoint(double x, double y);//set target point wo yaw, Monitoring of reaching it will be done in tick()
Odometry* odometryFromControl;

private:
Subscriber* subscriber;
double leftWheelSpeed;
    double rightWheelSpeed;
    double wheelBase;
    double wheelRadius;
    void calcWheelSpeeds();
    void sendWheelSpeeds();
};

#endif // MOTORCONTROL_H
