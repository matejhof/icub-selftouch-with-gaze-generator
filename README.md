# icub-selftouch-with-gaze-generator
To command the iCub simulator to points in Cartesian space where the end-effectors (palms or palm and finger) would intersect and the robot will simultaneously gaze at them. Logs all the corresponding joint configurations.

![alt text](https://github.com/matejhof/icub-selftouch-with-gaze-generator/blob/master/iCubSelfTouch.png "iCub self-touch")

iCubSelfTouch.png

The operation is illustrated in these videos:
- finger to palm (orientation unconstrained): https://youtu.be/ab-CgkLOlMw
- palm to palm: https://youtu.be/TX1qlbDYcIU

Note: The code has not been tested on the real robot. Some options - e.g. palm to palm configurations - are certainly **not to be used on the real robot!**

Note also that the solutions are a result of optimization and do not match perfectly with the desired target and hence also the 3 chains (2 arms, 1 gaze) do not intersect perfectly in 1 point in the operational space. 

## Requirements and operation
- `yarpserver`
- `iCub_SIM`

and as per: `app/scripts/icubSimSelfTouchWithGaze.xml`
- `yarprobotinterface --context simCartesianControl`
- `iKinCartesianSolver --context simCartesianControl --part left_arm`
- `iKinCartesianSolver --context simCartesianControl --part right_arm`
- `iKinGazeCtrl --from configSim.ini`

Then run `selftouchWithGazeGenerator`

## Parameters

The targets are chosen from a cube where both arms can reach and also where the robot can gaze at them.
In the iCub Root reference frame:
```c
#define TARGET_CUBE_MIN_X   -0.26
#define TARGET_CUBE_MAX_X   -0.21
#define TARGET_CUBE_MIN_Y   -0.1
#define TARGET_CUBE_MAX_Y    0.1
#define TARGET_CUBE_MIN_Z    0.0
#define TARGET_CUBE_MAX_Z    0.3 
```
There is a "grid search" through this cube with
```c
#define POINTS_PER_DIMENSION 24 //resolution of Cartesian grid; in reality, it will be this +1 in every dimension
```
E.g. for `POINTS_PER_DIMENSION 24`, 25 x 25 x 25, i.e. 15575 points will be sampled.

With this flag, a random sequence of the targets is followed (a permutation of the order - without repetition).
```c 
#define TARGET_SEQUENCE_RANDOM 1
```

The end-effector of the left arm is always the palm. For the right arm, tip of index finger can be chosen:
```c
#define USE_FINGER_ON_RIGHT_ARM 1
```
The user can also choose:
```c
#define VISUALIZE_TARGET_IN_ICUBSIM 1 //red sphere in icubSim marks the target
#define ASK_FOR_ARM_POSE_ONLY 1 //to ask for and log solutions for arm poses without commanding the simulator
```

The latter setting can be applied to the arm Cartesian controllers (see also http://www.icub.org/software_documentation/icub_cartesian_interface.html); however, this is not possible to apply to the gaze controller that has no such methods and the robot/simulator needs to be commanded (http://wiki.icub.org/iCub/main/dox/html/icub_gaze_interface.html).

```c
#define LOG_INTO_FILE 1
```

## Log file
The modules creates a `selfTouchConfigs.log` log file in the current directory with one row per self-touch configuration and the following structure:
- columns 1:3: target (x,y,z) (in metres, iCub Root reference frame)
- columns 4:13: left arm chain (3 torso, 7 arm joints) (deg) 
- columns 14:23: right arm chain (3 torso, 7 arm joints) (deg) 
- columns 24:29: head and eyes chain (neck pitch, roll, yaw, eyes tilt, eyes pan, eyes vergence)

## Kinematics version 
The `iCub_SIM` has kinematics version 1. 


If you want to use this module to generate solutions for kinematics V2, you can set the Cartesian solver to use V2 kinematics in:
`.local/share/yarp/contexts/simCartesianControl/cartesian/Left_arm_cartesian.xml` 

Change `<param name="KinematicType">left</param>` to `<param name="KinematicType">left_v2</param>` 

`Right_arm_cartesian.xml` 

Change `<param name="KinematicType">right</param>` to `<param name="KinematicType">right_v2</param>`

Then you have to use the `#define ASK_FOR_ARM_POSE_ONLY 1` regime.

However, it seems that in reality, V1 was still being used when tested.

For the gaze controller, it is not possible to choose the kinematics version.

## Cartesian solver accuracy
For this application, we changed in .local/share/yarp/contexts/simCartesianControl/cartesianSolver.ini
```
[left_arm]
tol            0.001 -> 0.0001
maxIter     200 ->   2000
[right_arm]
tol            0.001 -> 0.0001
maxIter     200 ->   2000
```


