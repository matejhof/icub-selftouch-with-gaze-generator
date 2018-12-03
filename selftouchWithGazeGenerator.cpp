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

//for random permutation, using http://www.cplusplus.com/reference/algorithm/random_shuffle/
#include <algorithm>    // std::random_shuffle
#include <vector>       // std::vector
#include <ctime>        // std::time
#include <cstdlib>      // std::rand, std::srand

//yarp and iCub stuff
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <iCub/ctrl/neuralNetworks.h>
#include <iCub/iKin/iKinFwd.h>

/* Datasets 1 and 2 - Nov 2017 and Aug 2018
#define TARGET_CUBE_MIN_X   -0.26
#define TARGET_CUBE_MAX_X   -0.21
#define TARGET_CUBE_MIN_Y   -0.05
#define TARGET_CUBE_MAX_Y    0.05
#define TARGET_CUBE_MIN_Z    0.0
#define TARGET_CUBE_MAX_Z    0.07
#define POINTS_PER_DIMENSION 19 //resolution of Cartesian grid; in reality, it will be this +1 in every dimension
*/

/* Datasets 3 and 4 - Aug and Nov 2018 */
#define TARGET_CUBE_MIN_X   -0.26
#define TARGET_CUBE_MAX_X   -0.21
#define TARGET_CUBE_MIN_Y   -0.1
#define TARGET_CUBE_MAX_Y    0.1
#define TARGET_CUBE_MIN_Z    0.0
#define TARGET_CUBE_MAX_Z    0.3

#define POINTS_PER_DIMENSION 24 //resolution of Cartesian grid; in reality, it will be this +1 in every dimension
#define VISUALIZE_TARGET_IN_ICUBSIM 1 //red sphere in icubSim marks the target
#define ASK_FOR_ARM_POSE_ONLY 0 //to ask for and log solutions for arm poses without commanding the simulator
#define LOG_INTO_FILE 1
#define TARGET_SEQUENCE_RANDOM 1 //if 0, grid with targets is covered systematically;
//if 1, the points on the grid are chosen following an initial permutation (no repetition of targets)

#define USE_FINGER_ON_RIGHT_ARM 1 //if 1, the right arm effector is shifted from palm to tip of index finger
#define USE_ORIENTATION 1//if 1, the right arm effector (palm/fingertip) will point into the left palm with a given orientation
// (perpendicular to the palm, but with all 3 dims given)

#define NR_ARM_JOINTS 7 //this is not to be changed by the user

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


class CtrlThread: public Thread
{
    protected:

    yarp::os::Port portToSimWorld;
    yarp::os::Bottle    cmd;
    ofstream fout_log;

    PolyDriver *drvCartLeftArm;
    PolyDriver *drvCartRightArm;
    PolyDriver *drvGazeCtrl;
    PolyDriver *drvLeftArm;
    PolyDriver *drvRightArm;

    ICartesianControl *cartCtrlLeftArm;
    ICartesianControl *cartCtrlRightArm;
    IGazeControl      *gazeCtrl;
    IEncoders *iencsLeftArm;
    IEncoders *iencsRightArm;
    IControlMode2     *modeRightArm;
    IPositionControl  *posRightArm;

    Vector armVels;
    iCub::iKin::iCubArm *lArm;
    iCub::iKin::iKinChain  *lArmChain;
    iCub::iKin::iCubArm *rArm;
    iCub::iKin::iKinChain  *rArmChain;
    iCub::iKin::iCubFinger *finger; //will be right arm index finger
    Vector pointingFingerHandPos;
    Vector handVels;

    yarp::sig::Matrix T_world_root; //homogenous transf. matrix expressing the rotation and translation of FoR from world (simulator) to from robot (Root) FoR

    Vector curCartDofLeft, newCartDofLeft, curCartDofRight, newCartDofRight;
    Vector xd; //target position
    Vector x_t;  // Current end-effector position

    double trajTime;
    double reachTol;

    struct Indexes3D
    {
     int indexI;
     int indexJ;
     int indexK;
    } indexes3D;

    vector<Indexes3D> indexesIJK; //combinations of all three - systematic or permutated (if TARGET_SEQUENCE_RANDOM 1)

    void close()
    {
        delete drvCartLeftArm;
        delete drvCartRightArm;
        delete drvGazeCtrl;
        delete drvLeftArm;
        delete drvRightArm;
        delete lArm;
        delete rArm;
        delete finger;

        if (portToSimWorld.isOpen())
        {
            portToSimWorld.interrupt();
            portToSimWorld.close();
        }

        fout_log.close();
    }


    void pointRightHand()
    {

        drvRightArm->view(modeRightArm);
        drvRightArm->view(posRightArm);

        for (size_t j=0; j<handVels.length(); j++)
            modeRightArm->setControlMode(armVels.length()+j,VOCAB_CM_POSITION);

        for (size_t j=0; j<handVels.length(); j++)
        {
            int k=armVels.length()+j;
            posRightArm->setRefSpeed(k,handVels[j]);
            posRightArm->positionMove(k,(pointingFingerHandPos)[j]);
        }
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
        reachTol = 0.0001; // m;
        string fwslash="/";
        string name="selftouchWithGazeGenerator";
        string robot="icubSim";

        // open cartesiancontrollerclient and gazecontrollerclient drivers
        Property optCartLeftArm("(device cartesiancontrollerclient)");
        Property optCartRightArm("(device cartesiancontrollerclient)");
        Property optGazeCtrl("(device gazecontrollerclient)");
        Property optLeftArm("(device remote_controlboard)");
        Property optRightArm("(device remote_controlboard)");

        optCartLeftArm.put("remote",(fwslash+robot+"/cartesianController/left_arm").c_str());
        optCartLeftArm.put("local",(fwslash+name+"/left_arm/cartesian").c_str());

        optCartRightArm.put("remote",(fwslash+robot+"/cartesianController/right_arm").c_str());
        optCartRightArm.put("local",(fwslash+name+"/right_arm/cartesian").c_str());

        optGazeCtrl.put("remote","/iKinGazeCtrl");
        optGazeCtrl.put("local",(fwslash+name+"/gaze").c_str());

        optLeftArm.put("remote",(fwslash+robot+"/left_arm").c_str());
        optLeftArm.put("local","/local/left_arm");

        optRightArm.put("remote",(fwslash+robot+"/right_arm").c_str());
        optRightArm.put("local","/local/right_arm");

        drvCartLeftArm=new PolyDriver;
        drvCartRightArm=new PolyDriver;
        drvGazeCtrl=new PolyDriver;
        drvLeftArm=new PolyDriver;
        drvRightArm=new PolyDriver;
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
        if (!drvLeftArm->open(optLeftArm))
        {
            close();
            return false;
        }
        if (!drvRightArm->open(optRightArm))
        {
            close();
            return false;
        }

        drvCartLeftArm->view(cartCtrlLeftArm);
        cartCtrlLeftArm->getDOF(curCartDofLeft);
        newCartDofLeft=curCartDofLeft;
        // to disable all torso joints
        newCartDofLeft[0]=0;
        newCartDofLeft[1]=0;
        newCartDofLeft[2]=0;
        // send the request for dofs reconfiguration
        cartCtrlLeftArm->setDOF(newCartDofLeft,curCartDofLeft);
        cartCtrlLeftArm->setTrackingMode(false);
        cartCtrlLeftArm->setTrajTime(trajTime);
        cartCtrlLeftArm->setInTargetTol(reachTol);
        // print out some info about the controller
        Bottle infoCartLA;
        cartCtrlLeftArm->getInfo(infoCartLA);
        fprintf(stdout,"Cart. controller left arm info = %s\n",infoCartLA.toString().c_str());

        drvCartRightArm->view(cartCtrlRightArm);
        cartCtrlRightArm->getDOF(curCartDofRight);
        newCartDofRight=curCartDofRight;
        // to disable all torso joints
        newCartDofRight[0]=0;
        newCartDofRight[1]=0;
        newCartDofRight[2]=0;
        // send the request for dofs reconfiguration
        cartCtrlRightArm->setDOF(newCartDofRight,curCartDofRight);
        cartCtrlRightArm->setTrackingMode(false);
        cartCtrlRightArm->setTrajTime(trajTime);
        cartCtrlRightArm->setInTargetTol(reachTol);
        // print out some info about the controller
        Bottle infoCartRA;
        cartCtrlRightArm->getInfo(infoCartRA);
        fprintf(stdout,"Cart. controller right arm info = %s\n",infoCartRA.toString().c_str());

        drvGazeCtrl->view(gazeCtrl);
        Bottle infoGaze;
        gazeCtrl->getInfo(infoGaze);
        fprintf(stdout,"Gaze controller info = %s\n",infoGaze.toString().c_str());

        drvLeftArm->view(iencsLeftArm);
        drvRightArm->view(iencsRightArm);

        armVels.resize(7,0.0);
        lArm = new iCub::iKin::iCubArm("left");
        lArmChain = lArm->asChain();
        rArm = new iCub::iKin::iCubArm("right");
        rArmChain = rArm->asChain();
        finger = new iCub::iKin::iCubFinger;
        pointingFingerHandPos.resize(9,0.0);
        // values for pointing hand: https://github.com/robotology/icub-main/blob/master/app/actionsRenderingEngine/conf/hand_sequences.ini#L26
        //60.0 30.0 18.0 86.0 5.0 1.0 70.0 100.0 200.0
        pointingFingerHandPos(0)=60.0; pointingFingerHandPos(1)=30.0; pointingFingerHandPos(2)=18.0;
        pointingFingerHandPos(3)=86.0; pointingFingerHandPos(4)=5.0; pointingFingerHandPos(5)=1.0;
        pointingFingerHandPos(6)=70.0; pointingFingerHandPos(7)=100.0; pointingFingerHandPos(8)=200.0;
        handVels.resize(9,10.0);
        if (USE_FINGER_ON_RIGHT_ARM)
            pointRightHand();

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

        //fill in the points on the grid - combinations of indexes i,j,k
        for(int i=0;i<=POINTS_PER_DIMENSION;i++)
         for(int j=0;j<=POINTS_PER_DIMENSION;j++)
          for(int k=0;k<=POINTS_PER_DIMENSION;k++)
          {
              indexes3D.indexI = i;
              indexes3D.indexJ = j;
              indexes3D.indexK = k;
              indexesIJK.push_back(indexes3D);
          }

        /*
        std::cout << "indexesIJK before shuffling:";
        for (std::vector<Indexes3D>::iterator it=indexesIJK.begin(); it!=indexesIJK.end(); ++it)
        {
            std::cout << ' ' << (*it).indexI << ' ' << (*it).indexJ << ' ' << (*it).indexK;
            std::cout << '\n';
        }
        std::cout << '\n \n';
        */

        if(TARGET_SEQUENCE_RANDOM)
        {
            //http://www.cplusplus.com/reference/algorithm/random_shuffle/
            std::srand ( unsigned ( std::time(0) ) );

            // using built-in random generator:
            std::random_shuffle ( indexesIJK.begin(), indexesIJK.end() );

            /*
            std::cout << "indexesIJK after shuffling:";
            for (std::vector<Indexes3D>::iterator it=indexesIJK.begin(); it!=indexesIJK.end(); ++it)
            {
                std::cout << ' ' << (*it).indexI << ' ' << (*it).indexJ << ' ' << (*it).indexK;
                std::cout << '\n';
            }
            std::cout << '\n \n';
            */
        }

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
        Vector x_d_sim(3,0.0);
        Vector xdhat_leftArm, odhat_leftArm, qdhat_leftArm;
        Vector xdhat_rightArm, odhat_rightArm, qdhat_rightArm;
        Vector qdhat_gaze; //desired gaze angles
        Vector x_gaze; //actual fixation point (not possible to obtain desired fixation point from gazeCtrl)
        double e_x_left, e_x_right; //error between target and desired Cartesian point returned by solver
        double e_x_gaze; //here error between target and actual fixation point
        int nEncsLeftArm, nEncsRightArm;
        while (isStopping() != true)
        { // the thread continues to run until isStopping() returns true

            if(VISUALIZE_TARGET_IN_ICUBSIM)
            {

                xd[0]= TARGET_CUBE_MIN_X + ((TARGET_CUBE_MAX_X-TARGET_CUBE_MIN_X)/2.0);
                xd[1]= TARGET_CUBE_MIN_Y + ((TARGET_CUBE_MAX_Y-TARGET_CUBE_MIN_Y)/2.0);
                xd[2]= TARGET_CUBE_MIN_Z + ((TARGET_CUBE_MAX_Z-TARGET_CUBE_MIN_Z)/2.0);
                convertPosFromRootToSimFoR(xd,x_d_sim);
                createStaticSphere(0.01,x_d_sim);
            }
            int counter = 1;
            for (std::vector<Indexes3D>::iterator it=indexesIJK.begin(); it!=indexesIJK.end(); ++it)
            {
                xd[0]= TARGET_CUBE_MIN_X + (*it).indexI*((TARGET_CUBE_MAX_X-TARGET_CUBE_MIN_X)/POINTS_PER_DIMENSION);
                xd[1]= TARGET_CUBE_MIN_Y + (*it).indexJ*((TARGET_CUBE_MAX_Y-TARGET_CUBE_MIN_Y)/POINTS_PER_DIMENSION);
                xd[2]= TARGET_CUBE_MIN_Z + (*it).indexK*((TARGET_CUBE_MAX_Z-TARGET_CUBE_MIN_Z)/POINTS_PER_DIMENSION);

                if(VISUALIZE_TARGET_IN_ICUBSIM)
                {
                    convertPosFromRootToSimFoR(xd,x_d_sim);
                    moveSphere(1,x_d_sim);
                }
                // go to the target
                fprintf(stdout,"CtrlThread:run(): Going to target: %.4f %.4f %.4f (%d out of %d)\n",xd[0],xd[1],xd[2],counter,indexesIJK.size());

                //reade encoders
                iencsLeftArm->getAxes(&nEncsLeftArm);
                Vector encsLeftArm(nEncsLeftArm);
                iencsLeftArm->getEncoders(encsLeftArm.data());
                iencsRightArm->getAxes(&nEncsRightArm);
                Vector encsRightArm(nEncsRightArm);
                iencsRightArm->getEncoders(encsRightArm.data());
                fprintf(stdout,"CtrlThread:run(): lefttArm/Hand motor encoders nEcns: %d, encs: %s\n",nEncsLeftArm,encsLeftArm.toString().c_str());
                fprintf(stdout,"CtrlThread:run(): rightArm/Hand motor encoders nEcns: %d, encs: %s\n",nEncsRightArm,encsRightArm.toString().c_str());

                //update chains
                lArm->setAng((encsLeftArm.subVector(0,NR_ARM_JOINTS-1))*CTRL_DEG2RAD); //take out the fingers; arm chain without torso joints here, encoders are arm only too
                rArm->setAng((encsRightArm.subVector(0,NR_ARM_JOINTS-1))*CTRL_DEG2RAD);

                //debugging only
                Vector qLA = lArm->getAng() * CTRL_RAD2DEG;
                fprintf(stdout,"CtrlThread:run(): lArm joints:\n%s\n", qLA.toString().c_str());
                Vector qRA = rArm->getAng() * CTRL_RAD2DEG;
                fprintf(stdout,"CtrlThread:run(): rArm joints:\n%s\n", qRA.toString().c_str());
                Matrix leftEEframe = lArm->getH();
                fprintf(stdout,"CtrlThread:run(): left arm EE frame:\n%s\n", leftEEframe.toString().c_str());
                Matrix rightEEframe = rArm->getH();
                fprintf(stdout,"CtrlThread:run(): right arm EE frame:\n%s\n", rightEEframe.toString().c_str());


                if (! cartCtrlLeftArm->askForPosition(xd,xdhat_leftArm,odhat_leftArm,qdhat_leftArm))
                    yInfo("  cartCtrlLeftArm->askForPosition could not find solution.");
                if (! ASK_FOR_ARM_POSE_ONLY)
                    cartCtrlLeftArm->goToPositionSync(xd);

                if(USE_FINGER_ON_RIGHT_ARM)
                {

                    fprintf(stdout,"CtrlThread:run(): Using right index finger as effector\n");
                    Vector rightIndexFingerJointValues;
                    finger->getChainJoints(encsRightArm,rightIndexFingerJointValues);   // wrt the end-effector frame
                    fprintf(stdout,"CtrlThread:run(): right arm finger chain joint values: %s\n", rightIndexFingerJointValues.toString().c_str());
                    Matrix tipFrame=finger->getH((M_PI/180.0)*rightIndexFingerJointValues);
                    fprintf(stdout,"CtrlThread:run(): right arm finger tip frame:\n%s\n", tipFrame.toString().c_str());
                    Vector tip_x=tipFrame.getCol(3);
                    Vector tip_o=yarp::math::dcm2axis(tipFrame);
                    cartCtrlRightArm->attachTipFrame(tip_x,tip_o);
                }
                if (USE_ORIENTATION)
                {
                   Vector lArmEEpose = lArm->EndEffPose(true);
                   fprintf(stdout,"CtrlThread:run(): left arm EE pose: %s\n", lArmEEpose.toString().c_str());
                   Vector lArmEEorientationAxisAngle = lArmEEpose.subVector(3,6);
                   Vector od = lArmEEorientationAxisAngle;
                   od(2)= lArmEEorientationAxisAngle(2) * -1.0; //we need the opposite direction
                   fprintf(stdout,"CtrlThread:run():  desired orientation right EE: %s\n", od.toString().c_str());
                   if (! cartCtrlRightArm->askForPose(xd,od,xdhat_rightArm,odhat_rightArm,qdhat_rightArm))
                       yInfo("  cartCtrlRightArm->askForPose could not find solution.");
                   else
                        fprintf(stdout,"CtrlThread:run():  odhat_rightArm: %s\n", odhat_rightArm.toString().c_str());
                   if (! ASK_FOR_ARM_POSE_ONLY)
                       cartCtrlRightArm->goToPoseSync(xd,od);
                }
                else
                {
                    if (! cartCtrlRightArm->askForPosition(xd,xdhat_rightArm,odhat_rightArm,qdhat_rightArm))
                        yInfo("  cartCtrlRightArm->askForPosition could not find solution.");
                    if (! ASK_FOR_ARM_POSE_ONLY)
                        cartCtrlRightArm->goToPositionSync(xd);
                }
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
                counter++;
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

