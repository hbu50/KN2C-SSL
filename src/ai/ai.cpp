#include "ai.h"

AI::AI(WorldModel *worldmodel, OutputBuffer *outputbuffer, QObject *parent) :
    QObject(parent),
    _wm(worldmodel),
    _outputbuffer(outputbuffer)
{
    qDebug() << "AI Initialization...";

}

void AI::Start()
{
    qDebug() << "AI::Start";

}

void AI::Stop()
{
    qDebug() << "AI::Stop";

}


