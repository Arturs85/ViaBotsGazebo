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


class Line2p{public: double x1; double y1;double x2;double y2;
         public :Line2p(double x1,double y1,double x2,double y2):
        x1(x1),y1(y1),x2(x2),y2(y2){}
            };

class ModelPush : public ModelPlugin
{
    std::vector<Line2p> lines;
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
private: transport::PublisherPtr lineSensorPublisher;
private: physics::WorldPtr world;

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


public: double distanceToLine(Line2p &line){
        double x = model->WorldPose().X();
        double y = model->WorldPose().Y();

        double A = x - line.x1;
        double B = y - line.y1;
        double C = line.x2 - line.x1;
        double D = line.y2 - line.y1;

        double dot = A * C + B * D;
        double len_sq = C * C + D * D;
        double param = -1;
        if (len_sq != 0) //in case of 0 length line
            param = dot / len_sq;

        double xx, yy;

        if (param < 0) {
            xx = line.x1;
            yy = line.y1;
        }
        else if (param > 1) {
            xx = line.x2;
            yy = line.y2;
        }
        else {
            xx = line.x1 + param * C;
            yy = line.y1 + param * D;
        }

        double dx = x - xx;
        double dy = y - yy;
        return std::sqrt(dx * dx + dy * dy);


    }
public: double findNearestLine(){
        if(lines.size()<1) return 0;
        double leastDistSoFar = 10000000;//todo double max
        int indexOfNearest =0;
        for (int i = 0; i < lines.size(); ++i) {
            double dist = distanceToLine(lines.at(i));
            if(dist<=leastDistSoFar){
                indexOfNearest = i;
                leastDistSoFar = dist;
            }
        }


    }
    // Returns 1 if the lines intersect, otherwise 0. In addition, if the lines
    // intersect the intersection point may be stored in the floats i_x and i_y.
public:   char get_line_intersection(Line2p l1,Line2p l2,
                                     float *i_x, float *i_y)
    {
        // float p0_x, float p0_y, float p1_x, float p1_y
        //float p2_x, float p2_y, float p3_x, float p3_y
        float s1_x, s1_y, s2_x, s2_y;
        s1_x = l1.x2 - l1.x1;     s1_y = l1.y2 - l1.y1;
        s2_x = l2.x2 - l2.x1;     s2_y = l2.y2 - l2.y1;

        float s, t;
        s = (-s1_y * (l1.x1 - l2.x1) + s1_x * (l1.y1 - l2.y1)) / (-s2_x * s1_y + s1_x * s2_y);
        t = ( s2_x * (l1.y1 - l2.y1) - s2_y * (l1.x1 - l2.x1)) / (-s2_x * s1_y + s1_x * s2_y);

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        {
            // Collision detected
            if (i_x != NULL)
                *i_x = l1.x1 + (t * s1_x);
            if (i_y != NULL)
                *i_y = l1.y1 + (t * s1_y);
            return 1;
        }

        return 0; // No collision
    }
public: Line2p getCurentSensorAbsLoction(){
        Line2p res(0,0,0,0);
        msgs::Geometry msg0;
        baseLink->GetCollision(0)->GetShape()->FillMsg(msg0);

        double blx = msg0.box().size().x();
        double bly = msg0.box().size().y(); //todo move to startup

        // std::cout<<" base lnk world pose: "<<baseLink->WorldPose()<< " blx: "<<blx<<" bly:"<<bly<<std::endl;

        //sensor absolute x1,y1
        double fi = std::atan2(bly/2,blx/2);
        double diogn = blx/2/std::cos(fi);

        res.x1 =diogn*std::cos(baseLink->WorldPose().Yaw()+fi)+baseLink->WorldPose().Pos().X();
        res.y1 =diogn*std::sin(baseLink->WorldPose().Yaw()+fi)+baseLink->WorldPose().Pos().Y();

        //double fi2 = baseLink->WorldPose().Yaw()+(M_PI/2-fi);
        res.x2 =diogn*std::cos(baseLink->WorldPose().Yaw()-fi)+baseLink->WorldPose().Pos().X();
        res.y2 =diogn*std::sin(baseLink->WorldPose().Yaw()-fi)+baseLink->WorldPose().Pos().Y();

        return res;
    }
public: double getLineSensorReading(Line2p sensor){
        double res = 100;// to line
        bool hasIntersection = false;
        float ix, iy;
        for (size_t i = 0; i < lines.size(); ++i) {
            hasIntersection = get_line_intersection(sensor,lines.at(i),&ix,&iy);
            if(hasIntersection){
                //find distance from left side of sensor
                double dx = sensor.x1-ix;
                double dy = sensor.y1-iy;

                double len = std::sqrt(dx*dx+dy*dy);
                dx = sensor.x1-sensor.x2;
                dy = sensor.y1-sensor.y2;

                double sensorLen = std::sqrt(dx*dx+dy*dy);

                res= sensorLen/2-len;
                std::cout<<" hasIntersevtion : "<<hasIntersection<<" "<<ix<<"  " <<iy<<std::endl;
                std::cout<<"line  p1: "<<lines.at(i).x1<<" "<<lines.at(i).y1<<" p2: "<<lines.at(i).x2<<" "<<lines.at(i).y2<<std::endl;
                std::cout<<"sens  p1: "<<sensor.x1<<" "<<sensor.y1<<" p2: "<<sensor.x2<<" "<<sensor.y2<<std::endl;
            }
        }

        return res;
    }

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // Store the pointer to the model
        this->model = _parent;
        this->world =  gazebo::physics::get_world();
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
        this->lineSensorPublisher = this->node->Advertise<msgs::Vector2d>("~/lineSensor");

        baseLink = links.at(0);
        //   baseLink->SetLinearVel(ignition::math::Vector3d(velX, 0.1, 0));
        msgs::Geometry msg0;
        baseLink->GetCollision(0)->GetShape()->FillMsg(msg0);

        double blx = msg0.box().size().x();
        double bly = msg0.box().size().y();

        std::cout<<" base lnk world pose: "<<baseLink->WorldPose()<< " blx: "<<blx<<" bly:"<<bly<<std::endl;

        //sensor absolute x1,y1
        double fi = std::atan2(bly/2,blx/2);
        double diogn = blx/2/std::cos(fi);

        double sx1 =diogn*std::cos(baseLink->WorldPose().Yaw()+fi)+baseLink->WorldPose().Pos().X();
        double sy1 =diogn*std::sin(baseLink->WorldPose().Yaw()+fi)+baseLink->WorldPose().Pos().Y();

        double fi2 = baseLink->WorldPose().Yaw()+(M_PI/2-fi);
        double sx2 =diogn*std::cos(baseLink->WorldPose().Yaw()-fi)+baseLink->WorldPose().Pos().X();
        double sy2 =diogn*std::sin(baseLink->WorldPose().Yaw()-fi)+baseLink->WorldPose().Pos().Y();

        std::cout<<" base lnk points p1 : "<<sx1<<" "<<sy1<<" p2: " <<sx2<<" "<<sy2<<std::endl;

        Line2p l1(1,0,1,10);
        Line2p l2(0,2,10,2);
        float ix,iy;
        bool hasIntersection = get_line_intersection(l1,l2,&ix,&iy);
        std::cout<<" hasIntersevtion : "<<hasIntersection<<" "<<ix<<"  " <<iy<<std::endl;


        auto models = world->Models();
        for (const auto &m : models)
        {

            if (m->GetName().find("yellow line") != std::string::npos)
            {
                printf(" model name: %s \n ",m->GetScopedName().c_str());


                printf(" line  x : %f ; y : %f \n ",m->RelativePose().Pos().X(),m->RelativePose().Pos().Y());
                msgs::Geometry msg;
                m->GetLink("link_ground")->GetCollision(0)->GetShape()->FillMsg(msg);
                double l = msg.plane().size().x();

                printf(" line shape size: %f \n ",msg.plane().size().x());


                double dx = l/2*std::cos(m->RelativePose().Yaw());
                double dy = l/2*std::sin(m->RelativePose().Yaw());
                double x2 =dx+m->RelativePose().Pos().X();
                double y2 =dy+m->RelativePose().Pos().Y();
                double x1 =-dx+m->RelativePose().Pos().X();
                double y1 =-dy+m->RelativePose().Pos().Y();


                lines.push_back(Line2p(x1,y1,x2,y2));
                //  targetPos=& m->RelativePose().Pos();
                //atjaunina waypoints sarakstu
                // if(std::find(waypoints.begin(), waypoints.end(), m) != waypoints.end()) {
                /* v contains x */
                // } else {
                /* v does not contain x */
                //     waypoints.push_back(m);
                // }

            }
            //revJoints.push_back(j);

        }for (int i = 0; i < lines.size(); ++i) {
            double dist = distanceToLine(lines.at(i));
            std::cout<<"line "<<i<<" p1: "<<lines.at(i).x1<<" "<<lines.at(i).y1<<" p2: "<<lines.at(i).x2<<" "<<lines.at(i).y2<<" distance: " <<dist<<std::endl;
        }

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

            Line2p sensor = getCurentSensorAbsLoction();
            // std::cout<<sensor.x1<<" "<<sensor.y1<<" "<<sensor.x2<<" "<<sensor.y2<<std::endl;
            double reading = getLineSensorReading(sensor);
            msgs::Vector2d msgLine;

            msgLine.set_x(reading);
            msgLine.set_y(reading);
            this->lineSensorPublisher->Publish(msgLine);
            std::cout<<"line sensor reading: "<<reading <<std::endl;
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

