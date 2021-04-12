#ifndef PUBLISHERBASE_H
#define PUBLISHERBASE_H
#include <vector>
#include <string>

class Listener{
public:
    virtual void callBack()=0;
    Listener();
    ~Listener();
};
class PublisherBase
{
public:
    PublisherBase();
   //void  subscribe(void(*_fp)(const std::string &));
    virtual void subscribe(Listener *listener){listeners.push_back(listener);}
protected:
   // std::vector<void(*)(const std::string &)> callbacks;
    std::vector<Listener*> listeners;
   virtual void makeAllCallbacks();
};

#endif // PUBLISHERBASE_H
