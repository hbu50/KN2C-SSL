#include "worldmodel.h"

WorldModel::WorldModel()
{
    time=0;
    refgs.cmd=0;
    gs = STATE_Null;
    gs_last = STATE_Null;

    for(int i=0; i<PLAYERS_MAX_NUM; i++)
    {
        ourRobot[i].id = i;
        oppRobot[i].id = i;
    }
}
