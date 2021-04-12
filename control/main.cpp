
#include <iostream>
#include "subscriber.h"
#include "trajectoryexecutor.h"
#include "pathplanner.h"
#include "particlefilter.h"
#include <QApplication>
#include <pthread.h>


/* thread function */
void *thr_func(void *arg) {


    printf("started new thread");
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

int main(int _argc, char **_argv){

    QApplication qap(_argc,_argv);
    std::cout<<"main.cpp main executed  "<<std::endl;

    pthread_t threadId;

    // Create a thread that will function threadFunc()
    int err = pthread_create(&threadId, NULL, &thr_func, NULL);
    // Check if thread is created sucessfuly
    if (err)
    {
        std::cout << "thread creation failed : " << strerror(err);
        return err;
    }
    else
        std::cout << "thread Created with ID : " << threadId << std::endl;

    qap.exec();






}
