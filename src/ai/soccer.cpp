#include "soccer.h"

Soccer::Soccer(QObject *parent) :
    QObject(parent)
{
    qDebug() << "Soccer Initialization...";
    Settings s("settings.ini");

    QString gm = s.Get("Game","Mode");
    QString rip = s.Get(gm, "RefIP");
    int rport = s.Get(gm, "RefPort").toInt();
    QString vip = s.Get(gm, "VisionIP");
    int vport = s.Get(gm, "VisionPort").toInt();
    QString scolor = s.Get("Team","Color");
    TeamColorType tcolor = (scolor=="Blue")?COLOR_BLUE:COLOR_YELLOW;
    QString sside = s.Get("Team","Side");
    TeamSideType tside = (sside=="Right")?SIDE_RIGHT:SIDE_LEFT;
    CameraConfigType tcam = (CameraConfigType)s.Get("VisionConfig","UsingCameras").toInt();
    QString simip = s.Get("grSim", "CommandIP");
    int simport = s.Get("grSim", "CommandPort").toInt();
    QString serialport = s.Get("Transmitter","SerialPort");
    float ball_dist = s.Get("RefereeConfig", "RefereeBall").toFloat();

    // Game mode
    gamemode = (gm=="Simulation")?MODE_SIMULATION:MODE_REAL;
    qDebug() << "Game mode: " << gm << "\n";

    // output buffer
    outputbuffer = new OutputBuffer;

    // Vison & referee
    wm = new WorldModel(outputbuffer);
    sslvision = new SSLVision(vip, vport, tcolor, tside, tcam, wm);
    sslrefbox = new SSLRefBox(rip, rport, tcolor, ball_dist, wm);
    sslvision->Start();
    sslrefbox->Start();

    // grSim
    if(gamemode==MODE_SIMULATION)
    {
        grsim = new grSim(simip, simport, tcolor, outputbuffer);
        grsim->Start();
    }

    // serial port transmitter
    if(gamemode==MODE_REAL)
    {
        transmitter = new Transmitter(serialport, outputbuffer);
        transmitter->Start();
    }

    // AI
    ai = new AI(wm, outputbuffer, this);
    ai->Start();
}
