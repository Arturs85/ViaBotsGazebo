#ifndef LINEFOLLOWER_H
#define LINEFOLLOWER_H


class LineFollower
{
public:
    LineFollower();
    void tick();
   bool foundLine = false;

  double static getYaw(double sensorReading);

};

#endif // LINEFOLLOWER_H
