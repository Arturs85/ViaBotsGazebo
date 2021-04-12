#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
# include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
//#include "odometry.h"
namespace gazebo
{
class ModelPush : public ModelPlugin
{
    physics::LinkPtr baseLink;
    physics::JointPtr rightWheel;
    physics::JointPtr leftWheel;

    std::string leftWheelName = "left_wheel_hinge";
    std::string rightWheelName = "right_wheel_hinge";
    /// \brief Node used to establish communication with gzserver.
    transport::NodePtr node;
    /// \brief Subscriber to world statistics messages.
    transport::SubscriberPtr statsSub;
    /// \brief Publisher of factory messages.
private: transport::PublisherPtr odoPublisher;
private: transport::PublisherPtr gpsPublisher;


    // Odometry odo;
    double wheelSpeedRight=0;
    double wheelSpeedLeft=0;


    int c=0;
    bool hasReceivedControlMsg = false;
public: void OnControlMsg(ConstVector2dPtr &_msg){
        if(!hasReceivedControlMsg)hasReceivedControlMsg=true;
        if(c%100==0){
            std::cout<<"msg arrived "<<_msg->x()<<std::endl;
        }
        c++;
        wheelSpeedLeft=_msg->x();
        wheelSpeedRight = _msg->y();

    }



public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                    std::bind(&ModelPush::OnUpdate, this));

        physics::Link_V links = this->model->GetLinks();
        physics::Joint_V joints = this->model->GetJoints();

        std::cout<<"model plugin : model link count: "<<links.size()<<"\n";
        std::cout<<"model plugin : model joint count: "<<joints.size()<<"\n";
        for (physics::JointPtr j : joints) {
            std::cout<<"model joint name : "<<  j->GetName()<<"\n";
            if(j->GetName().find(leftWheelName)!=std::string::npos){leftWheel = j;   std::cout<<"left wheel found: "<<j->GetName()<<"\n";}
            if(j->GetName().find(rightWheelName)!=std::string::npos){rightWheel = j; std::cout<<"right wheel found: "<<j->GetName()<<"\n";}



        }
        // Create a node for transportation
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init("default");
        this->statsSub = this->node->Subscribe("~/motorControlSub", &ModelPush::OnControlMsg,this);
        this->odoPublisher = this->node->Advertise<msgs::Vector2d>("~/odometry");
        this->gpsPublisher = this->node->Advertise<msgs::Vector2d>("~/gps");

        baseLink = links.at(0);
        //   baseLink->SetLinearVel(ignition::math::Vector3d(velX, 0.1, 0));


    }



    double velX=1;
    // Called by the world update start event
    int counter =0;


public: void OnUpdate()
    {
        //try to recconect to control message publisher until messages arrive
        if(!hasReceivedControlMsg)
            this->statsSub = this->node->Subscribe("~/motorControlSub", &ModelPush::OnControlMsg,this);

        counter++;
        // Apply a small linear velocity to the model.
        //   this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
        //  velX =baseLink->RelativeLinearVel().X() ;
        if(leftWheel!=NULL)
            leftWheel->SetVelocity(0,wheelSpeedLeft);
        if(rightWheel!=NULL)
            rightWheel->SetVelocity(0,wheelSpeedRight);


        //odo.updateAngles(M_PI*leftWheel->Position()/180,M_PI*rightWheel->Position()/180);
        // odo.updateAngles(leftWheel->Position(),rightWheel->Position());

        if(counter%100==0){
            //  std::cout<<"x: "<<odo.x<<"  y: "<<odo.y<<std::endl;
            msgs::Vector2d msg;

            msg.set_x(leftWheel->Position());
            msg.set_y(rightWheel->Position());
            this->odoPublisher->Publish(msg);

            if(counter%1000==0){//gps msg, todo - make time vased intervals instead of iteration count based ones

                msgs::Vector2d gpsMsg;

                gpsMsg.set_x( model->WorldPose().X());
                gpsMsg.set_y(model->WorldPose().Y());
                gpsPublisher->Publish(gpsMsg);
            }

            //std::cout<<"lwp: "<<leftWheel->Position()<<"  rwp: "<<rightWheel->Position()<<std::endl;
        }

        //if(baseLink->WorldPose().X()<-1)
        //  velX= - velX;
        //if(model->RelativePose().Pos().X()>1){
        //   velX= - velX;
        // this->model->SetLinearVel(ignition::math::Vector3d(velX, 0.2, 0));

        //}
        //baseLink->SetLinearVel(ignition::math::Vector3d(velX, 0.1, 0));

    }

    // Pointer to the model
private: physics::ModelPtr model;

    // Pointer to the update event connection
private: event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}

