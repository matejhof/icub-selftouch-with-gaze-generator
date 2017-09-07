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
#include <iostream>
#include <fstream>
#include <iomanip>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/neuralNetworks.h>

#define TARGET_CUBE_MIN_X   -0.26
#define TARGET_CUBE_MAX_X   -0.21
#define TARGET_CUBE_MIN_Y   -0.05
#define TARGET_CUBE_MAX_Y    0.05
#define TARGET_CUBE_MIN_Z    0.0
#define TARGET_CUBE_MAX_Z    0.07
#define VISUALIZE_TARGET_IN_ICUBSIM 1
#define ASK_FOR_ARM_POSE_ONLY 1
#define LOG_INTO_FILE 1

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

class CtrlThread: public Thread
{
    protected:

    ICartesianControl *cartCtrlLeftArm;
    ICartesianControl *cartCtrlRightArm;
    IGazeControl      *gazeCtrl;
    PolyDriver *drvCartLeftArm;
    PolyDriver *drvCartRightArm;
    PolyDriver *drvGazeCtrl;

    yarp::os::Port portToSimWorld;
    yarp::os::Bottle    cmd;

    ofstream fout_log;

    yarp::sig::Matrix T_world_root; //homogenous transf. matrix expressing the rotation and translation of FoR from world (simulator) to from robot (Root) FoR

    Vector curDofLeft, newDofLeft, curDofRight, newDofRight;
    Vector xd; //target position

    double trajTime;
    double reachTol;

    void close()
    {
        delete drvCartLeftArm;
        delete drvCartRightArm;
        delete drvGazeCtrl;

        if (portToSimWorld.isOpen())
        {
            portToSimWorld.interrupt();
            portToSimWorld.close();
        }


        fout_log.close();
    }

    /***** visualizations in iCub simulator ********************************/

   void createStaticSphere(double radius, const Vector &pos)
   {
       cmd.clear();
       cmd.addString("world");
       cmd.addString("mk");
       cmd.addString("ssph");
       cmd.addDouble(radius);

       cmd.addDouble(pos(0));
       cmd.addDouble(pos(1));
       cmd.addDouble(pos(2));
       // color
       cmd.addInt(1);cmd.addInt(0);cmd.addInt(0);
       cmd.addString("false"); //no collisions
       yInfo("createSphere(): sending %s \n",cmd.toString().c_str());
       portToSimWorld.write(cmd);
   }

   void moveSphere(int index, const Vector &pos)
   {
       cmd.clear();
       cmd.addString("world");
       cmd.addString("set");
       cmd.addString("ssph");
       cmd.addInt(index);
       cmd.addDouble(pos(0));
       cmd.addDouble(pos(1));
       cmd.addDouble(pos(2));
       portToSimWorld.write(cmd);
   }


   void convertPosFromRootToSimFoR(const Vector &pos, Vector &outPos)
   {
       Vector pos_temp = pos;
       pos_temp.resize(4);
       pos_temp(3) = 1.0;

       //printf("convertPosFromRootToSimFoR: need to convert %s in icub root FoR to simulator FoR.\n",pos.toString().c_str());
       //printf("convertPosFromRootToSimFoR: pos in icub root resized to 4, with last value set to 1:%s\n",pos_temp.toString().c_str());

       outPos.resize(4,0.0);
       outPos = T_world_root * pos_temp;
       //printf("convertPosFromRootToSimFoR: outPos in simulator FoR:%s\n",outPos.toString().c_str());
       outPos.resize(3);
       //printf("convertPosFromRootToSimFoR: outPos after resizing back to 3 values:%s\n",outPos.toString().c_str());
       return;
   }

   public:

    CtrlThread(): Thread()
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

        trajTime = 0.3; //seconds
        reachTol = 0.001; // m;
        string fwslash="/";
        string name="selftouchWithGazeGenerator";
        string robot="icubSim";

        // open cartesiancontrollerclient and gazecontrollerclient drivers
        Property optCartLeftArm("(device cartesiancontrollerclient)");
        Property optCartRightArm("(device cartesiancontrollerclient)");
        Property optGazeCtrl("(device gazecontrollerclient)");

        optCartLeftArm.put("remote",(fwslash+robot+"/cartesianController/left_arm").c_str());
        optCartLeftArm.put("local",(fwslash+name+"/left_arm/cartesian").c_str());

        optCartRightArm.put("remote",(fwslash+robot+"/cartesianController/right_arm").c_str());
        optCartRightArm.put("local",(fwslash+name+"/right_arm/cartesian").c_str());

        optGazeCtrl.put("remote","/iKinGazeCtrl");
        optGazeCtrl.put("local",(fwslash+name+"/gaze").c_str());

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

        drvCartLeftArm->view(cartCtrlLeftArm);
        cartCtrlLeftArm->getDOF(curDofLeft);
        newDofLeft=curDofLeft;
        // to disable all torso joints
        newDofLeft[0]=0;
        newDofLeft[1]=0;
        newDofLeft[2]=0;
        // send the request for dofs reconfiguration
        cartCtrlLeftArm->setDOF(newDofLeft,curDofLeft);
        cartCtrlLeftArm->setTrackingMode(false);
        cartCtrlLeftArm->setTrajTime(trajTime);
        cartCtrlLeftArm->setInTargetTol(reachTol);
        // print out some info about the controller
        Bottle infoLA;
        cartCtrlLeftArm->getInfo(infoLA);
        fprintf(stdout,"Cart. controller left arm info = %s\n",infoLA.toString().c_str());

        drvCartRightArm->view(cartCtrlRightArm);
        cartCtrlRightArm->getDOF(curDofRight);
        newDofRight=curDofRight;
        // to disable all torso joints
        newDofRight[0]=0;
        newDofRight[1]=0;
        newDofRight[2]=0;
        // send the request for dofs reconfiguration
        cartCtrlRightArm->setDOF(newDofRight,curDofRight);
        cartCtrlRightArm->setTrackingMode(false);
        cartCtrlRightArm->setTrajTime(trajTime);
        cartCtrlRightArm->setInTargetTol(reachTol);
        // print out some info about the controller
        Bottle infoRA;
        cartCtrlRightArm->getInfo(infoRA);
        fprintf(stdout,"Cart. controller right arm info = %s\n",infoRA.toString().c_str());

        drvGazeCtrl->view(gazeCtrl);
        Bottle infoGaze;
        gazeCtrl->getInfo(infoGaze);
        fprintf(stdout,"Gaze controller info = %s\n",infoGaze.toString().c_str());

        T_world_root = zeros(4,4);
        T_world_root(0,1)=-1;
        T_world_root(1,2)=1; T_world_root(1,3)=0.5976;
        T_world_root(2,0)=-1; T_world_root(2,3)=-0.026;
        T_world_root(3,3)=1;


        if (LOG_INTO_FILE)
        {
            yInfo("Opening log file..");
            fout_log.open("selfTouchConfigs.log");
        }

        if(VISUALIZE_TARGET_IN_ICUBSIM)
        {
            string port2icubsim = "/" + name + "/sim:o";
            if (!portToSimWorld.open(port2icubsim.c_str())) {
                yError("[selftouchWithGazeGenerator] Unable to open port << port2icubsim << endl");
            }
            std::string port2world = "/icubSim/world";
            yarp::os::Network::connect(port2icubsim, port2world.c_str());

            cmd.clear();
            cmd.addString("world");
            cmd.addString("del");
            cmd.addString("all");
            portToSimWorld.write(cmd);
        }

        xd.resize(3); //target position

        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Thread started successfully\n");
        else
            fprintf(stdout,"Thread did not start\n");
    }

    virtual void run()
    {

        int pointsPerDimension = 19; //in reality, it will be this +1
        Vector x_d_sim(3,0.0);
        Vector xdhat_leftArm, odhat_leftArm, qdhat_leftArm;
        Vector xdhat_rightArm, odhat_rightArm, qdhat_rightArm;
        Vector qdhat_gaze; //desired gaze angles
        Vector x_gaze; //actual fixation point (not possible to obtain desired fixation point from gazeCtrl)
        double e_x_left, e_x_right; //error between target and desired Cartesian point returned by solver
        double e_x_gaze; //here error between target and actual fixation point
        while (isStopping() != true)
        { // the thread continues to run until isStopping() returns true
            if(VISUALIZE_TARGET_IN_ICUBSIM)
            {

                xd[0]= TARGET_CUBE_MIN_X + ((TARGET_CUBE_MAX_X-TARGET_CUBE_MIN_X)/2.0);
                xd[1]= TARGET_CUBE_MIN_Y + ((TARGET_CUBE_MAX_Y-TARGET_CUBE_MIN_Y)/2.0);
                xd[2]= TARGET_CUBE_MIN_Z + ((TARGET_CUBE_MAX_Z-TARGET_CUBE_MIN_Z)/2.0);
                convertPosFromRootToSimFoR(xd,x_d_sim);
                createStaticSphere(0.03,x_d_sim);
            }
            //prepare grid to sample points
            for(int i=0;i<=pointsPerDimension;i++)
             for(int j=0;j<=pointsPerDimension;j++)
              for(int k=0;k<=pointsPerDimension;k++)
              {
                    xd[0]= TARGET_CUBE_MIN_X + i*((TARGET_CUBE_MAX_X-TARGET_CUBE_MIN_X)/pointsPerDimension);
                    xd[1]= TARGET_CUBE_MIN_Y + j*((TARGET_CUBE_MAX_Y-TARGET_CUBE_MIN_Y)/pointsPerDimension);
                    xd[2]= TARGET_CUBE_MIN_Z + k*((TARGET_CUBE_MAX_Z-TARGET_CUBE_MIN_Z)/pointsPerDimension);
                    if(VISUALIZE_TARGET_IN_ICUBSIM)
                    {
                        convertPosFromRootToSimFoR(xd,x_d_sim);
                        moveSphere(1,x_d_sim);
                    }
                   // go to the target
                   fprintf(stdout,"CtrlThread:run(): Going to target: %.4f %.4f %.4f\n",xd[0],xd[1],xd[2]);

                   if (! cartCtrlLeftArm->askForPosition(xd,xdhat_leftArm,odhat_leftArm,qdhat_leftArm))
                       yInfo("  cartCtrlLeftArm->askForPosition could not find solution.");
                   if (! ASK_FOR_ARM_POSE_ONLY)
                       cartCtrlLeftArm->goToPositionSync(xd);

                   if (! cartCtrlRightArm->askForPosition(xd,xdhat_rightArm,odhat_rightArm,qdhat_rightArm))
                       yInfo("  cartCtrlRightArm->askForPosition could not find solution.");
                   if (! ASK_FOR_ARM_POSE_ONLY)
                       cartCtrlRightArm->goToPositionSync(xd);

                   if(! gazeCtrl->lookAtFixationPointSync(xd))
                       yInfo("  gazeCtrl could not find solution.");
                   gazeCtrl->getJointsDesired(qdhat_gaze);

                   if (! ASK_FOR_ARM_POSE_ONLY)
                   {
                       if (! cartCtrlLeftArm->waitMotionDone(0.04))
                           yInfo("  cartCtrlLeftArm could not reach solution.");
                       if (! cartCtrlRightArm->waitMotionDone(0.04))
                           yInfo("  cartCtrlRightArm could not reach solution.");
                   }
                   if (! gazeCtrl->waitMotionDone(0.04,0.0))
                        yInfo("  gazeCtrl could not reach solution.");

                   // we get the current arm position in the operational space
                   //cartCtrlLeftArm->getPose(x,o);

                   e_x_left=norm(xdhat_leftArm-xd);
                   //fprintf(stdout,"++left arm+++\n");
                   //fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
                   //fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
                   //fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
                   fprintf(stdout," left arm (target vs. solver): norm(e_x)   [m] = %g\n",e_x_left);
                   e_x_right=norm(xdhat_rightArm-xd);
                   fprintf(stdout," right arm: (target vs. solver) norm(e_x)   [m] = %g\n",e_x_right);
                   gazeCtrl->getFixationPoint(x_gaze);
                   e_x_gaze=norm(x_gaze-xd);
                   fprintf(stdout," gaze ctrl: (target vs. actual) norm(e_x)   [m] = %g\n",e_x_gaze);

                   yInfo("%s",xd.toString().c_str());
                   yInfo("%s",qdhat_leftArm.toString().c_str());
                   yInfo("%s",qdhat_rightArm.toString().c_str());
                   yInfo("%s",qdhat_gaze.toString().c_str());
                   //yInfo("%s %s %s %s",xd.toString.c_str(),qdhat_leftArm.toString().c_str(),qdhat_rightArm.toString().c_str(),qdhat_gaze.toString().c_str());
                   if(LOG_INTO_FILE)
                       fout_log<<setprecision(4)<<xd.toString().c_str()<<" "<<qdhat_leftArm.toString().c_str()<<" "<<qdhat_rightArm.toString().c_str()<<" "<<qdhat_gaze.toString().c_str()<<endl;

              }
           stop();
        }

    }



    virtual void threadRelease()
    {
        // we require an immediate stop
        // before closing the client for safety reason
        fprintf(stdout,"threadRelease: Cart control left arm: stopping control.\n");
        cartCtrlLeftArm->stopControl();
        fprintf(stdout,"threadRelease: Cart control right arm: stopping control.\n");
        cartCtrlRightArm->stopControl();
        fprintf(stdout,"CtrlThread::threadRelease: gaze control: stopping control.\n");
        gazeCtrl->stopControl();

        if(VISUALIZE_TARGET_IN_ICUBSIM)
        {
            fprintf(stdout,"Deleting objects from simulator world.\n");
            cmd.clear();
            cmd.addString("world");
            cmd.addString("del");
            cmd.addString("all");
            portToSimWorld.write(cmd);
        }

        close();
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

            thr=new CtrlThread();
            thr->start();

            return true;
        }

        virtual bool close()
        {

            printf("CtrlModule:close(): will stop thread.\n");
            thr->stop();
            printf("CtrlModule:close(): called thr->stop(), will delete thr.\n");
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

