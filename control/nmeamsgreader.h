#ifndef NMEAMSGREADER_H
#define NMEAMSGREADER_H
#include <string>


class NmeaMsgReader
{
public:
   double positionTimeSec=0;
   double headingTimeSec=0;
   double la=0; //latitude degrees
   double lo =0; //longitude degrees
double heading =0; //degrees
std::string rmcSentenceName = "$GNRMC";
NmeaMsgReader();
    void addNmeaString(std::string msg);
    void positionReceived();
static double minuteToDecimal(std::string m);
static double nmeaTimeToSeconds(std::string t);

void testWithFile();
};

#endif // NMEAMSGREADER_H
