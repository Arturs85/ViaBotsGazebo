#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/gazebo_client.hh>
#include "subscriber.h"





int Subscriber::counter =0;
int Subscriber::c =0;
double Subscriber::lineSensorReading = 100;
Odometry Subscriber::odo;
GpsPublisher Subscriber::gpsPublisher;

transport::NodePtr Subscriber::node;
transport::SubscriberPtr Subscriber::statsSub;
transport::SubscriberPtr Subscriber::gpsSub;
transport::SubscriberPtr Subscriber::lineSub;

transport::PublisherPtr Subscriber::wheeSpeedPub;


void Subscriber::OnControlMsg(ConstVector2dPtr &_msg){
    if(c%1==0){
        std::cout<<"msg arrived "<<_msg->x()<<std::endl;
    }
    c++;
    if(_msg->x()>0){
        // force = 0.7;
    }else{
        //    force = -0.4;


    }

}

void Subscriber::OnOdometryMsg(ConstVector2dPtr &_msg){

    // std::cout<<"odom msg "<<_msg->x()<<" "<<_msg->y()<<std::endl;
    odo.updateAngles(_msg->x(),_msg->y());
}

void Subscriber::OnLineSensorMsg(ConstVector2dPtr &_msg)
{
    lineSensorReading = _msg->x();
}

void Subscriber::sendWheelSpeeds(double l, double r){

    msgs::Vector2d msg;
    msg.set_x(l);
    msg.set_y(r);
    //  wheeSpeedPub.get()->
    wheeSpeedPub->Publish(msg);
    //std::cout<<"subscriber: wheel speeds sent\n";
}

void Subscriber::OnGpsMsg(ConstVector2dPtr &_msg)
{
   // std::cout<<"subscriber gps msg received<<\n";
    gpsPublisher.gpsCallback(_msg->x(),_msg->y());
}

int Subscriber::init(){

    gazebo::client::setup();

    // Create a node for transportation
    node = transport::NodePtr(new transport::Node());
    node->Init("default");
    statsSub = node->Subscribe("~/odometry", &Subscriber::OnOdometryMsg);
    gpsSub = node->Subscribe("~/gps", &Subscriber::OnGpsMsg);
    lineSub = node->Subscribe("~/lineSensor", &Subscriber::OnLineSensorMsg);

    wheeSpeedPub = node->Advertise<msgs::Vector2d>("~/motorControlSub");
    while(wheeSpeedPub.get()->GetRemoteSubscriptionCount()<1){//wait while we hav subscriber
        usleep(10000);

    }

    return 0;
}
