#ifndef PATHPLANNER_H
#define PATHPLANNER_H
#include "trajectoryexecutor.h"
#include <vector>

class PathPlanner
{
public:
    PathPlanner();
Position2D getNextPosition();

private:
std::size_t currentTargretPointIndex=0;
TrajectoryExecutor te;
std::vector<Position2D> pathPoints;
std::vector<Position2D> pathPositons;

Position2D getNextPointAfterTarget();//to calculate position of arrival

};

#endif // PATHPLANNER_H
