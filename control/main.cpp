
#include <iostream>
#include "subscriber.h"
#include "trajectoryexecutor.h"
#include "pathplanner.h"
#include "particlefilter.h"
#include <QApplication>

int main(int _argc, char **_argv){
QApplication qap(_argc,_argv);
    std::cout<<"main.cpp main executed  "<<std::endl;
   // Subscriber s;
Subscriber::init();
std::cout<<"subscriber init executed "<<std::endl;

TrajectoryExecutor trajectoryExecutor;
PathPlanner pathPalnner;
ParticleFilter particleFilter(&Subscriber::odo);
trajectoryExecutor.setTarget(0.3,0,-4);
std::cout<<"trajestoryexec set target executed "<<std::endl;

while(true){
   if( trajectoryExecutor.tick()){
   Position2D nextPoint = pathPalnner.getNextPosition();
       trajectoryExecutor.setTarget(0.3,nextPoint.x, nextPoint.y);
   }
   // std::cout<<"trajestoryexec tick executed "<<std::endl;

    usleep(100000);


}



}
