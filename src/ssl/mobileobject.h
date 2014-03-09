#ifndef MOBILEOBJECT_H
#define MOBILEOBJECT_H

#include <QDebug>

#include "geom.h"
#include "position.h"

class MobileObject
{
public:
    explicit MobileObject();
    double time;
    bool isValid;
    Position pos;
    Position vel;

    void seenAt(Position p, double t, int camera);

};

#endif // MOBILEOBJECT_H
