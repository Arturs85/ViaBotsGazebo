#include "nmeamsgreader.h"
#include <vector>
#include<sstream>
#include <iostream>
#include <fstream>

NmeaMsgReader::NmeaMsgReader()
{
    testWithFile();
}

void NmeaMsgReader::addNmeaString(std::string msg)
{
    //read line by line
    //spilt message in to sentences
    std::vector<std::string> rmcFields;
    std::stringstream ss(msg);
    std::string s;
    while ( std::getline( ss, s ) ) {
        //  std::cout<<s.substr(0,6)<<"\n";
        if(s.compare(0,6,rmcSentenceName)==0){
            std::stringstream sentence(s);

            while ( std::getline( sentence, s,',' ) ) {
                rmcFields.push_back(s);
            }
            la = minuteToDecimal(rmcFields.at(3));
            lo = minuteToDecimal(rmcFields.at(5));
            positionTimeSec = nmeaTimeToSeconds(rmcFields.at(1));
            if(rmcFields.at(8).size()>1)//double always contains decimal point?
            { heading = std::stod(rmcFields.at(8));
                headingTimeSec = positionTimeSec;
            }
            else heading =0.0;
            std::cout<<s<<"   nrOfFields: "<<rmcFields.size()<<" lat: "<<la<<" long: "<< lo<<" heading: "<<heading<<" time_sec: "<<positionTimeSec<<"\n";
        }
    }
}

//convert degrees from minutes representation found in nmea to decimal representation for easier computation
// short version - does not take in to account N/S,E/W
double NmeaMsgReader::minuteToDecimal(std::string m)
{
    size_t p = m.find('.');

    double deg = std::stoi(m.substr(0,p-2));
    double min = std::stod(m.substr(p-2,std::string::npos));

    return deg+min/60;

}

double NmeaMsgReader::nmeaTimeToSeconds(std::string t)
{
    try {
        int h = std::stoi(t.substr(0,2));
        int m = std::stoi(t.substr(2,2));
        double s = std::stod(t.substr(4,std::string::npos));
        return h*3600+m*60+s;

    } catch (std::invalid_argument ia) {
        std::cout<<"could not convert "<<t<<" "<<ia.what()<<std::endl;
    }
    return -1;
}



void NmeaMsgReader::testWithFile()
{
    std::string lastLine;
    std::ifstream inFile("/home/arturs/rs2HeadingTest");
    while (std::getline(inFile, lastLine)){
        addNmeaString(lastLine);

    }
}
