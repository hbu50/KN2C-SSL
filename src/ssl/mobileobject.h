#ifndef MOBILEOBJECT_H
#define MOBILEOBJECT_H

#include <QDebug>
#include <QTimer>
#include <vector>
using namespace std;

#include "geom.h"
#include "position.h"

class MobileObject : public QObject
{
    Q_OBJECT

public:
    explicit MobileObject();
    void seenAt(vector<Position> p, double t, int camera);

    double time;
    bool isValid;
    Position pos;
    Position vel;

private:
    QTimer timer;
    int interval;

private slots:
    void timer_timeout();

};

#endif // MOBILEOBJECT_H
