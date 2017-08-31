// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// To command the iCub simulator to points in Cartesian space where the end-effectors (palms)
// would intersect and the robot will simultaneously gaze at them.
// Note: do not use on the real robot!
// Author: Matej Hoffmann - <matej.hoffmann@fel.cvut.cz>

// Based on several resources by Ugo Pattacini
// in particular:
//   https://github.com/robotology/icub-basic-demos/tree/master/demoRedBall
//   http://wiki.icub.org/brain/tutorial__cartesian__interface_8cpp.html


#include <cstdio>
#include <cmath>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/neuralNetworks.h>

#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

class CtrlThread: public RateThread
{
    protected:

    ICartesianControl *icart;
    IGazeControl      *gazeCtrl;
    PolyDriver *drvCartLeftArm, *drvCartRightArm;
    PolyDriver *drvGazeCtrl;

    Vector curDof, newDof;

    Vector xd;

    double trajTime;
    double reachTol;

    double t;
    double t0;
    double t1;

    void close()
    {
        delete drvCartLeftArm;
        delete drvCartRightArm;
        delete drvGazeCtrl;
    }


    public:

    CtrlThread(int _period): RateThread(_period)
    {
        ;
    }

    virtual bool threadInit()
    {
         // open a client interface to connect to the cartesian server and gaze interface of the simulator
         // we suppose that:
         // 1 - the iCub simulator is running (launch: iCub_SIM)
         // 2 - the cartesian server is running
         //     (launch: yarprobotinterface --context simCartesianControl)
         // 3 - the cartesian solver for the left arm is running too
         //     (launch: iKinCartesianSolver --context simCartesianControl --part left_arm)
         // 4 - the cartesian solver for the right arm is running too
         //     (launch: iKinCartesianSolver --context simCartesianControl --part right_arm)
         // 5 - the gaze interface is running
         //     (launch: iKinGazeCtrl --from configSim.ini)

        trajTime = 0.5; //seconds
        reachTol = 0.00001; // m; we put a very small one for the sake of calibration
        string fwslash="/";
        string name="selftouchWithGazeGenerator";
        string robot="icubSim";

        // open cartesiancontrollerclient and gazecontrollerclient drivers
        Property optCartLeftArm("(device cartesiancontrollerclient)");
        Property optCartRightArm("(device cartesiancontrollerclient)");
        Property optGazeCtrl("(device gazecontrollerclient)");

        optCartLeftArm.put("remote",(fwslash+robot+"/cartesianController/left_arm").c_str());
        optCartLeftArm.put("local",(name+"/left_arm/cartesian").c_str());

        optCartRightArm.put("remote",(fwslash+robot+"/cartesianController/right_arm").c_str());
        optCartRightArm.put("local",(name+"/right_arm/cartesian").c_str());

        optGazeCtrl.put("remote","/iKinGazeCtrl");
        optGazeCtrl.put("local",(name+"/gaze").c_str());


        drvCartLeftArm=new PolyDriver;
        drvCartRightArm=new PolyDriver;
        drvGazeCtrl=new PolyDriver;
        if (!drvCartLeftArm->open(optCartLeftArm))
        {
            close();
            return false;
        }

        if (!drvCartRightArm->open(optCartRightArm))
        {
            close();
            return false;
        }

        if (!drvGazeCtrl->open(optGazeCtrl))
        {
            close();
            return false;
        }

        drvCartLeftArm->view(icart);
        drvCartRightArm->view(icart);

        icart->getDOF(curDof);
        newDof=curDof;
        // to disable all torso joints
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;
        // send the request for dofs reconfiguration
        icart->setDOF(newDof,curDof);

        icart->setTrackingMode(false);
        icart->setTrajTime(trajTime);
        icart->setInTargetTol(reachTol);

        // print out some info about the controller
        Bottle info;
        icart->getInfo(info);
        fprintf(stdout,"info = %s\n",info.toString().c_str());

        xd.resize(3);

        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Thread started successfully\n");
        else
            fprintf(stdout,"Thread did not start\n");

        t=t0=t1=Time::now();
    }

    virtual void run()
    {
        t=Time::now();

        generateTarget();

        // go to the target
        icart->goToPositionSync(xd);
        //icart->goToPose(xd,od); // (in streaming)
        icart->waitMotionDone(0.04);

        // some verbosity
         printStatus();
    }



    virtual void threadRelease()
    {
        // we require an immediate stop
        // before closing the client for safety reason
        icart->stopControl();
        gazeCtrl->stopControl();

        close();
    }

    void generateTarget()
    {
        // translational target part: a circular trajectory
        // in the yz plane centered in [-0.3,0.0,0.1] with radius=0.1 m
        // and frequency 0.1 Hz
        xd[0]=-0.3;
        xd[1]=0.1*cos(2.0*M_PI*0.1*(t-t0));
        xd[2]=0.1+0.1*sin(2.0*M_PI*0.1*(t-t0));

        // we keep the orientation of the left arm constant:
        // we want the middle finger to point forward (end-effector x-axis)
        // with the palm turned down (end-effector y-axis points leftward);
        // to achieve that it is enough to rotate the root frame of pi around z-axis
        //od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;
    }

    void printStatus()
    {
        if (t-t1>=PRINT_STATUS_PER)
        {
            Vector x,o,xdhat,odhat,qdhat;

             // we get the current arm pose in the
             // operational space
             icart->getPose(x,o);

             // we get the final destination of the arm
             // as found by the solver: it differs a bit
             // from the desired pose according to the tolerances
             icart->getDesired(xdhat,odhat,qdhat);

             double e_x=norm(xdhat-x);
             double e_o=norm(odhat-o);

             fprintf(stdout,"+++++++++\n");
             fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
             fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
             fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
             fprintf(stdout,"norm(e_x)   [m] = %g\n",e_x);
             fprintf(stdout,"---------\n\n");

             t1=t;
         }
   }
};

class CtrlModule: public RFModule
 {
    protected:
        CtrlThread *thr;

    public:
        virtual bool configure(ResourceFinder &rf)
        {
            Time::turboBoost();

            thr=new CtrlThread(CTRL_THREAD_PER);
            if (!thr->start())
            {
                delete thr;
                return false;
            }

           return true;
        }

        virtual bool close()
        {
            thr->stop();
            delete thr;

            return true;
        }

        virtual double getPeriod()    { return 1.0;  }
        virtual bool   updateModule() { return true; }
 };

int main()
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return 1;
    }

    CtrlModule mod;

    ResourceFinder rf;
    return mod.runModule(rf);
}

