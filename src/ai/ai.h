#ifndef AI_H
#define AI_H

#include <QtCore>
#include <QObject>
#include "constants.h"
#include "worldmodel.h"
#include "outputbuffer.h"

class AI : public QObject
{
    Q_OBJECT
public:
    explicit AI(WorldModel* worldmodel, OutputBuffer* outputbuffer, QObject *parent = 0);

public slots:
    void Start();
    void Stop();

private:
    WorldModel* _wm;
    OutputBuffer* _outputbuffer;
    QTimer _timer;

};

#endif // AI_H
