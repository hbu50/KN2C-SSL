#include "mobileobject.h"

MobileObject::MobileObject() :
    QObject(0),
    time(0),
    isValid(false)
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(timer_timeout()));
    interval = 1000;
}

void MobileObject::timer_timeout()
{
    isValid = false;
}

void MobileObject::seenAt(vector<Position> p, double t, int camera)
{
    if(p.size()<1) return;
    Q_UNUSED(camera);
    time = t;
    isValid = true;
    timer.start(interval);

    int min_i = 0;
    double min_d = (pos.loc - p[0].loc).length2();

    for(size_t i=0; i< p.size(); i++)
    {
        auto d = (pos.loc - p[i].loc).length2();
        if(d < min_d)
        {
            min_d = d;
            min_i = i;
        }
    }

    pos.loc = pos.loc + (p[min_i].loc - pos.loc) * 0.6;
    pos.dir = pos.dir + (p[min_i].dir - pos.dir) * 0.8;
}


