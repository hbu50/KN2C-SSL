#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <QObject>
#include <QtCore>
#include <QDebug>
#include <QMap>
#include <math.h>
#include <cmath>
using namespace std;

#include "position.h"
#include "util.h"

struct ControllerInput
{
    Position cur_pos;
    Position cur_vel;

    Position mid_pos;
    Position mid_vel;

    Position fin_pos;
    Position fin_vel;

    char angleMode;
    double maxSpeed;
};

struct RobotSpeed
{
    int RID;
    float VX;
    float VY;
    float VW;
};

struct MotorSpeed
{
    float M0;
    float M1;
    float M2;
    float M3;
};

struct ControllerResult
{
    RobotSpeed rs;
    MotorSpeed msR;
    MotorSpeed msS;
};

class Controller : public QObject
{
    Q_OBJECT

public:
    explicit Controller(QObject *parent = 0);
    ControllerResult calc(ControllerInput &ci);

private:
    QTime timer;

    RobotSpeed calcRobotSpeed_main(ControllerInput &ci);
    RobotSpeed calcRobotSpeed_adjt(ControllerInput &ci);
    RobotSpeed calcRobotSpeed_test(ControllerInput &ci);

    MotorSpeed calcReal(RobotSpeed rs);
    MotorSpeed calcSimul(RobotSpeed rs);

};

#endif // CONTROLLER_H
