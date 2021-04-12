#include "particlefilter.h"
#include "odometry.h"
#include <iostream>
#include <algorithm>
#include <random>
#include "subscriber.h"

ParticleFilter::ParticleFilter(Odometry* odometry):OdometryListener(odometry)
{
    initializeParticles(0,0);
    odometry->subscribe((OdometryListener*)this);
    Subscriber::gpsPublisher.subscribe((GpsListener*)this);
guiWindow.updateView(1,1,1);//test
    guiWindow.show();
}

void ParticleFilter::onOdometry(Position2D position){
    //  std::cout<<"particleFilter onOdometry called "<<position.x<<std::endl;
moveParticles(position.x,position.y,position.yaw);

guiWindow.updateView(position.x,position.y,position.yaw);
}
void ParticleFilter::onGps(double x, double y){
    std::cout<<"particleFilter onGps called "<<x<<" "<<y<<std::endl;

}

void ParticleFilter::moveParticles(double dx, double dy, double dyaw)
{
    for (int i = 0; i < particles.size(); i++) {
        double fi = atan2(dy,dx);
        double dxg = dx*cos(fi+particles.at(i).direction)/cos(fi);
        double dyg = dy*sin(fi+particles.at(i).direction)/sin(fi);
        particles.at(i).x+=dxg;
        particles.at(i).y+=dyg;
        particles.at(i).direction+=dyaw;
    }
}

void ParticleFilter::calcFitness(double xGps, double yGps)
{
    double distanceSum =0;
    double longestDistance =0;
    for (int i = 0; i < particles.size(); i++) {
        Particle* p = & (particles.at(i));

        p->fitness= sqrt((xGps-p->x)*(xGps-p->x)+(yGps-p->y)*(yGps-p->y));//calc distance
        distanceSum += p->fitness;//store largest distance
        if(p->fitness>longestDistance) longestDistance = p->fitness;
        //if(p->fitness>GPS_DIST_ERR)p->isValid=false;
    }
    double avgDist = distanceSum/particles.size();
    for (int i = 0; i < particles.size(); i++) {
        particles.at(i).fitness = avgDist/particles.at(i).fitness;
    }
    //todo calc amount of fitness for one descendant, iterate trough particles, if fitnes > min add new particle, decrease fit by amount

}

void ParticleFilter::regenerateParticles()
{
    std::sort(particles.begin(), particles.end());// sort by fitness, at this point fitness should point, how much descendants particle should have
    std::vector<Particle> particlesRegenerated;
    for (int i = particles.size()-1; i >= 0; i--) {
        int descendantCount =   ((int)(particles.at(i).fitness))+1;// round up
        //create descendants
        for (int j = 0; j < descendantCount; ++j) {
            particlesRegenerated.push_back(particles.at(i));//todo distribute descendants spatially
            if(particlesRegenerated.size()>=PARTICLE_COUNT)break;

        }
        if(particlesRegenerated.size()>=PARTICLE_COUNT)break;

    }
    particles = particlesRegenerated;// should we copy data  or adress only?
}

void ParticleFilter::addMovmentNoise()
{
    for (int i = 0; i < particles.size(); i++) {

        std::default_random_engine generator;
        std::normal_distribution<double> distribution(0.0,0.1);// stddev value?
        std::normal_distribution<double> angledistribution(0.0,M_PI/4);// stddev value?

        double errx = std::abs(distribution(generator));
        double erry = std::abs(distribution(generator));
        double errYaw = (angledistribution(generator));

        particles.at(i).x-=errx;//remove only, because wheel slip cant produce larger distance than measured by odo?
        particles.at(i).y-=erry;
        particles.at(i).direction+=errYaw;

    }
}

double ParticleFilter::calcAverageDirection()
{
    double dir=0;
    for (int i = 0; i < particles.size(); i++) {
        dir+=particles.at(i).direction;

    }
    dir = dir/particles.size();
    return dir;
}



void ParticleFilter::initializeParticles(int x, int y) {
    particles.clear();
    for (int i = 0; i < PARTICLE_COUNT; i++) {


        particles.push_back( Particle(x, y, (rand() % 360)*M_PI / 180 ));

    }

}


Particle::Particle(double x, double y, double yaw)
{
    this->x=x;
    this->y = y;
    this->direction = yaw;
}
