#include "agent.h"
#include "worldmodel.h"

Agent::Agent() :
    Robot()
{
}

void Agent::setID(int id)
{
    this->id = id;
}

void Agent::setOutputBuffer(OutputBuffer *outputBuffer)
{
    this->outputBuffer = outputBuffer;
}

void Agent::setWorldModel(WorldModel *wm)
{
    this->wm = wm;
}

void Agent::SendCommand(RobotCommand rc)
{
    Position mypos=wm->ourRobot[id].pos;
    Position myvel=wm->ourRobot[id].vel;

    ControllerInput ci;
    ci.cur_pos = mypos;
    ci.cur_vel = myvel;

    ci.fin_pos = rc.fin_pos;
    ci.maxSpeed = rc.maxSpeed;

    ControllerResult ctrlres = ctrl.calc(ci);

    // Real Game Packet
    RobotData reRD;
    reRD.RID = id;
    reRD.M1 = ctrlres.msR.M1;
    reRD.M2 = ctrlres.msR.M2;
    reRD.M3 = ctrlres.msR.M3;
    reRD.M4 = ctrlres.msR.M4;
    reRD.KCK = (rc.kickspeedx>0)?1:0;
    reRD.CHP = 0;
    outputBuffer->wpck.AddRobot(reRD);

    // grSim Packet
    grRobotData grRD;
    grRD.rid=id;
    grRD.velx = ctrlres.rs.VX;
    grRD.vely = ctrlres.rs.VY;
    grRD.velw = ctrlres.rs.VW;
    grRD.wheel1=ctrlres.msS.M1;
    grRD.wheel2=ctrlres.msS.M2;
    grRD.wheel3=ctrlres.msS.M3;
    grRD.wheel4=ctrlres.msS.M4;
    grRD.kickspeedx=rc.kickspeedx;
    grRD.kickspeedz=rc.kickspeedz;
    grRD.spinner=0;
    outputBuffer->grpck.AddRobot(grRD);
}

void Agent::Halt()
{
    // Real Game Packet
    RobotData reRD;
    reRD.RID = id;
    reRD.M1 = 0;
    reRD.M2 = 0;
    reRD.M3 = 0;
    reRD.M4 = 0;
    reRD.KCK = 0;
    //reRD.FLG = 0;
    //reRD.ASK = 0;
    outputBuffer->wpck.AddRobot(reRD);
    // grSim Packet
    grRobotData grRD;
    grRD.rid=id;
    grRD.velx = 0;
    grRD.vely = 0;
    grRD.velw = 0;
    grRD.wheel1=0;
    grRD.wheel2=0;
    grRD.wheel3=0;
    grRD.wheel4=0;
    grRD.kickspeedx=0;
    grRD.kickspeedz=0;
    grRD.spinner=0;
    outputBuffer->grpck.AddRobot(grRD);
}

