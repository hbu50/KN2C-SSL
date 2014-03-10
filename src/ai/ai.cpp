#include "ai.h"

AI::AI(WorldModel *worldmodel, OutputBuffer *outputbuffer, QObject *parent) :
    QObject(parent),
    wm(worldmodel),
    outputbuffer(outputbuffer)
{
    qDebug() << "AI Initialization...";
    connect(&timer, SIGNAL(timeout()), this, SLOT(timer_timeout()));

}

void AI::Start()
{
    qDebug() << "AI::Start";
    timer.start(AI_TIMER);
}

void AI::Stop()
{
    qDebug() << "AI::Stop";

}

void AI::timer_timeout()
{
    RobotCommand rc;
    rc.fin_pos.loc = {0,0};
    rc.maxSpeed = 1;
    wm->ourRobot[0].SendCommand(rc);
}
