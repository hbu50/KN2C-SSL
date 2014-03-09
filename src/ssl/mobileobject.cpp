#include "mobileobject.h"

MobileObject::MobileObject() :
    time(0),
    isValid(false)
{
}

void MobileObject::seenAt(Position p, double t, int camera)
{
    qDebug() << "MobileObject::seenAt";
    pos = p;
    time = t;
    isValid = true;
}

