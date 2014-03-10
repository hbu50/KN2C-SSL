#include "controller.h"
#include "constants.h"

#define ROBOTRADIUS 0.090
#define SpeedToRPM 1375.14
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
    //double MAXROBOTSPEED = 2;MAXMOTORSRPM * M_PI * WHEELDIAMETER * cos(M_PI / 6) / 60;
       double MAXROTATIONSPEED = 4;//MAXMOTORSRPM * WHEELDIAMETER / (60 * ROBOTRADIUS * 2);
       //double ROBOTSPEED;
       double RotationSpeed,Rspeed_filer=0;

       //static Vector2D last_LinearSpeed;
       //static float last_RotationSpeed;
       static Vector2D err1,err0;
       static Vector2D u1;

       //static double Kp;
       //static double Kd;
       //static double delT = 0.020;
       //static double Ki;
       double ap=1;
       double am=1,amb=1;

       double t0;
       //static double t1;
       double s0;
        double s3;
        double s1;
        double v,vb = ci.maxSpeed/2.0;
       // double dt,s,sp,t2;
        //double tp;
        //double t2p;
        //double t3;
       // Position targetVel;
       ci.fin_vel.loc = Vector2D(0,0);

       //static Vector2D integral;
       //static int Integral_CNT;
       //static Vector2D derived;

       static double werr1;
       //static double werr0;
       static double wu1;

        double wKp;
        double wKd;
        double wKi;
       static double wintegral;
       static double wderived;





       /******************************Linear Speed Controller************************************/

       err0 = err1;
       err1 = (ci.fin_pos.loc - ci.cur_pos.loc)*.001;
        double dist;
       if (ci.cur_vel.loc.length()<vb)
       {
           dist = (pow(ci.fin_vel.loc.length(),2)-pow(ci.cur_vel.loc.length(),2))/(-2.0*amb);
       }
       else
       {

           dist =(pow(vb,2)-pow(ci.cur_vel.loc.length(),2))/(-2.0*am);
           dist+=(pow(ci.fin_vel.loc.length(),2)-pow(vb,2))/(-2.0*amb);


       }
       u1 = err1;
       if(err1.length()<=dist)
       {
           if(ci.cur_vel.loc.length()<vb)
               u1.setLength(sqrt(2.0*amb*(err1.length())+pow(ci.fin_vel.loc.length(),2)));
           else
               u1.setLength(sqrt(2.0*am*(err1.length())+pow(ci.fin_vel.loc.length(),2)));
       }
       else if(err1.length()>dist)
       {
           t0 = -ci.cur_vel.loc.length()/ap;
           //t1 = (vmax/ap)+t0;
           s0 = -ci.cur_vel.loc.length()*t0/2;
           s3 = pow(ci.fin_vel.loc.length(),2)/(2*am);
           s1 = (err1.length()+s0+s3)/(1+ap/am);
           v = sqrt(s1*2*ap);
           //tp = (v/ap)+t0;
           //t3 = (v/am) + tp;
           //t2p = t3 - (ci.fin_vel.loc.length()/am);
           //t2 = t2p;
           double Sm = (pow(v,2)-pow(ci.fin_vel.loc.length(),2))/(2.0*am);
           u1.setLength(sqrt(2.0*ap*((err1.length()-Sm))+pow(v,2)));

       }
       if(err1.length()<.010)
       {
           u1.setLength(0);
       }
       if(u1.length()>ci.maxSpeed)
       {
           u1.setLength(ci.maxSpeed);
       }

       Vector2D LinearSpeed = u1;
       Vector2D RotLinearSpeed=LinearSpeed;
       RotLinearSpeed.x = LinearSpeed.x * cos(ci.cur_pos.dir) + LinearSpeed.y * sin(ci.cur_pos.dir);
       RotLinearSpeed.y = -LinearSpeed.x * sin(ci.cur_pos.dir) + LinearSpeed.y * cos(ci.cur_pos.dir);

       /******************************Rotation Speed Controller************************************/
       wKp = 3.0;
       wKd = 0;
       wKi = 0;
       //werr0 = werr1;
       werr1 = ci.fin_pos.dir - ci.cur_pos.dir;
       if (werr1 > M_PI) werr1 -= 2 * M_PI;
       if (werr1 < -M_PI) werr1 += 2 * M_PI;
       wu1 = (werr1*wKp) + (wintegral*wKi) + wderived*wKd;
       Rspeed_filer=Rspeed_filer+(0.2*(wu1-Rspeed_filer));
       if (Rspeed_filer>MAXROTATIONSPEED) Rspeed_filer=MAXROTATIONSPEED;
       if (Rspeed_filer<-MAXROTATIONSPEED) Rspeed_filer=-MAXROTATIONSPEED;
       RotationSpeed = Rspeed_filer ;

       RobotSpeed ans;

       ans.VX = RotLinearSpeed.x;
       ans.VY = RotLinearSpeed.y;
       ans.VW = RotationSpeed;

       return ans;
}

MotorSpeed Controller::calcReal(RobotSpeed rs)
{
    double motor[4][1],rotate[4][3],speed[3][1];

    speed[0][0] = -rs.VX;
    speed[1][0] = -rs.VY;
    speed[2][0] = -rs.VW;

    rotate[0][0] =  cos( 0.18716 * M_PI);//cos(M_PI /4.0);//-sin(rangle + M_PI);//7/4
    rotate[1][0] =  sin( M_PI / 4.0 );//-cos(0.22 * M_PI);//-sin(rangle - M_PI / 3);//0.218
    rotate[2][0] =  -cos( M_PI / 4.0 );//-sin(0.22 * M_PI);//-sin(rangle + M_PI / 3);//0.78
    rotate[3][0] =  -cos( 0.18716 * M_PI);//cos(M_PI /4.0);//-sin(rangle + M_PI);//5/4
    rotate[0][1] =  -sin(0.18716 * M_PI );//cos(M_PI /4.0);//cos(rangle + M_PI);//7/4
    rotate[1][1] = cos(M_PI / 4.0 );//- sin(0.22 * M_PI);// cos(rangle - M_PI / 3);//0.218
    rotate[2][1] = sin(M_PI / 4.0);//cos(0.22 * M_PI);//cos(rangle + M_PI / 3);//0.187
    rotate[3][1] = -sin(0.18716 * M_PI);//-cos(M_PI /4.0);//cos(rangle + M_PI);//5/4

    rotate[0][2] = -ROBOTRADIUS;
    rotate[1][2] = -ROBOTRADIUS;
    rotate[2][2] = -ROBOTRADIUS;
    rotate[3][2] = -ROBOTRADIUS;

    motor[0][0] = (rotate[0][0] * speed[0][0] + rotate[0][1] * speed[1][0] + rotate[0][2] * speed[2][0])*SpeedToRPM;
    motor[1][0] = (rotate[1][0] * speed[0][0] + rotate[1][1] * speed[1][0] + rotate[1][2] * speed[2][0])*SpeedToRPM;
    motor[2][0] = (rotate[2][0] * speed[0][0] + rotate[2][1] * speed[1][0] + rotate[2][2] * speed[2][0])*SpeedToRPM;
    motor[3][0] = (rotate[3][0] * speed[0][0] + rotate[3][1] * speed[1][0] + rotate[3][2] * speed[2][0])*SpeedToRPM;

    //double MaxMotorSpeed = 3.47;//MAXMOTORSRPM * M_PI * WHEELDIAMETER / 60000;

    MotorSpeed result;

    result.M1 = (motor[0][0] /40.0);
    result.M2 = (motor[1][0] /40.0);
    result.M3 = (motor[2][0] /40.0);
    result.M4 = (motor[3][0] /40.0);

    double max = max4(fabs(result.M1),fabs(result.M2),fabs(result.M3),fabs(result.M4));

    if(max > 127)
    {
        result.M1 = (int)((result.M1 * 127.0)/max);
        result.M2 = (int)((result.M2 * 127.0)/max);
        result.M3 = (int)((result.M3 * 127.0)/max);
        result.M4 = (int)((result.M4 * 127.0)/max);
    }

    return result;
}

MotorSpeed Controller::calcSimul(RobotSpeed rs)
{
    double motor[4][1],rotate[4][3],speed[3][1];

    speed[0][0] = rs.VX;
    speed[1][0] = rs.VY;
    speed[2][0] = rs.VW;

    rotate[0][0] = sin(M_PI / 3);//-sin(rangle - M_PI / 3);
    rotate[1][0] = sin(3 * M_PI / 4);//-sin(rangle + M_PI / 3);
    rotate[2][0] = sin(5 * M_PI / 4);//-sin(rangle + M_PI);
    rotate[3][0] = sin(5 * M_PI / 3);//-sin(rangle + M_PI);
    rotate[0][1] = -cos(M_PI / 3);//cos(rangle - M_PI / 3);
    rotate[1][1] = -cos(3 * M_PI / 4);//cos(rangle + M_PI / 3);
    rotate[2][1] = -cos(5 * M_PI / 4);//cos(rangle + M_PI);
    rotate[3][1] = -cos(5 * M_PI / 3);//cos(rangle + M_PI);
    rotate[0][2] = -ROBOTRADIUS;
    rotate[1][2] = -ROBOTRADIUS;
    rotate[2][2] = -ROBOTRADIUS;
    rotate[3][2] = -ROBOTRADIUS;

    motor[0][0] = rotate[0][0] * speed[0][0] + rotate[0][1] * speed[1][0] + rotate[0][2] * speed[2][0];
    motor[1][0] = rotate[1][0] * speed[0][0] + rotate[1][1] * speed[1][0] + rotate[1][2] * speed[2][0];
    motor[2][0] = rotate[2][0] * speed[0][0] + rotate[2][1] * speed[1][0] + rotate[2][2] * speed[2][0];
    motor[3][0] = rotate[3][0] * speed[0][0] + rotate[3][1] * speed[1][0] + rotate[3][2] * speed[2][0];


    MotorSpeed result;

    result.M1 = -motor[0][0]*100;
    result.M2 = -motor[1][0]*100;
    result.M3 = -motor[2][0]*100;
    result.M4 = -motor[3][0]*100;

    return result;
}
