#ifndef SUBSCRIBER_H
#define SUBSCRIBER_H
# include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "odometry.h"
#include "publisherbase.h"

using namespace gazebo;

class Subscriber
{
public:   static  Odometry odo;
static GpsPublisher gpsPublisher;
    /// \brief Node used to establish communication with gzserver.
 static   transport::NodePtr node;
    /// \brief Subscriber to world statistics messages.
 static    transport::SubscriberPtr statsSub;
 static    transport::SubscriberPtr gpsSub;
 static    transport::SubscriberPtr lineSub;

 /// \brief Publisher of factory messages.

private: static transport::PublisherPtr wheeSpeedPub;
  //  double force;
public: static void OnControlMsg(ConstVector2dPtr &_msg);

public: static void  OnOdometryMsg(ConstVector2dPtr &_msg);
public: static void  OnLineSensorMsg(ConstVector2dPtr &_msg);

 public: static void  sendWheelSpeeds(double l, double r);

public: static void  OnGpsMsg(ConstVector2dPtr &_msg);


public:  static int  init();



   //double velX=1;
    // Called by the world update start event
    static int  counter;
    static int c;
static double lineSensorReading;
};

#endif // SUBSCRIBER_H
