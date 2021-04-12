#ifndef ODOMETRY_H
#define ODOMETRY_H
#include "trajectoryexecutor.h"//for Pose, replace with pose later- move pose to separate file
#include "publisherbase.h"

class Odometry: public PublisherBase
{
public:

    double WHEEL_RADI = 0.1;
    double WHEEL_BASE = 0.26;

    Odometry();
    double angleLeftCumulative =0;
    double angleRightCumulative =0;
    double angleLeftLastRead =0;
    double angleRightLastRead =0;
    double dt =0;
    double linearVelocity =0;
    Position2D pose;


    void updateAngles(double left, double right);
    void updateAnglesFromSpeed(double leftSpeed, double rightSpeed);
    double getLinearVelocity();

    void updatePose();


private:
    double getDeltaLeftAngleSinceLastRead();
    double getDeltaRightAngleSinceLastRead();

    double prevSpeedUpdateTime=0; //initialized in constructor with valid time



};

class OdometryListener: public Listener{// todo move class to own file?
public:
    OdometryListener(Odometry* odometry);
    void callBack();
    ~OdometryListener();
protected:
   virtual void onOdometry(Position2D position)=0;
private:
    Odometry* odo;
};
class GpsListener: public Listener{// todo move class to own file
public:
    void callBack();
~GpsListener();

    virtual void onGps(double x, double y)=0;
};
class GpsPublisher: public PublisherBase{
public: void gpsCallback(double x, double y);

};
#endif // ODOMETRY_H
