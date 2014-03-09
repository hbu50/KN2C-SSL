#include "controller.h"
#include "constants.h"

Controller::Controller(QObject *parent) :
    QObject(parent)
{
    qDebug() << "Controller Initialization...";
    timer.start();
}

ControllerResult Controller::calc(ControllerInput &ci)
{
    ControllerResult ctrlresult;
    RobotSpeed rs = calcRobotSpeed(ci);
    ctrlresult.msR = calcReal(rs);
    ctrlresult.msS = calcSimul(rs);
    return ctrlresult;
}

RobotSpeed Controller::calcRobotSpeed(ControllerInput &ci)
{
    int t = timer.elapsed();
    // ...
    RobotSpeed result;
    return result;
}

MotorSpeed Controller::calcReal(RobotSpeed rs)
{
    MotorSpeed result;
    return result;
}

MotorSpeed Controller::calcSimul(RobotSpeed rs)
{
    MotorSpeed result;
    return result;
}
