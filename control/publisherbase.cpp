#include "publisherbase.h"

PublisherBase::PublisherBase()
{

}

void PublisherBase::makeAllCallbacks()
{
    for (Listener* l : listeners) {
        l->callBack();
    }
}


Listener::Listener()
{

}

Listener::~Listener()
{

}
