#include "pathplanner.h"

PathPlanner::PathPlanner()
{
    pathPoints.push_back(Position2D(1,1,0));
    pathPoints.push_back(Position2D(-1,1,0));
    pathPoints.push_back(Position2D(1,-1,0));
    pathPoints.push_back(Position2D(-1,-1,0));


}

Position2D PathPlanner::getNextPosition()
{
    currentTargretPointIndex++;
    if(currentTargretPointIndex>=pathPoints.size()) currentTargretPointIndex =0;

    Position2D p1 = pathPoints.at(currentTargretPointIndex);
    Position2D p2 = getNextPointAfterTarget();
    double angle = p1.calcYawPointToPoint(p2);
    return Position2D(p1.x,p1.y,angle);
}

Position2D PathPlanner::getNextPointAfterTarget()
{
    std::size_t index = currentTargretPointIndex+1;
    if(index>=pathPoints.size()) index =0;
    return pathPoints.at(index);
}
