
## HOW TO RUN A SIMULATION WITH TORQUE CONTROL ON ICUB

The procedure to run the torque balancing module is still quite elaborate. Users willing to use the module should follow this list.

- Set the environmental variable YARP_ROBOT_NAME in the `.bashrc` according to the robot one wants to use (e.g. icubGazeboSim for simulations, or iCubGenova04, etc. for experiments).

- Launch yarpserver (with --write option if necessary).

- Launch gazebo. If you want to use the synchronization between the controller and the simulator to avoid real-time factor related problems, launch gazebo as follows: `gazebo -slibgazebo_yarp_clock.so`.
 
- Bring the robot in a suitable home position (e.g. `$ yarpmotorgui --from homePoseBalancing.ini` and then select a custom position by clicking on `Global Joints Commands/Custom postions`.

- Launch `wholeBodyDynamics` as follows: `YARP_ROBOT_NAME=icubGazeboSim yarprobotinterface --config launch-wholebodydynamics.xml`. For further details see [here](https://github.com/robotology/codyco-modules/blob/master/doc/force_control_on_icub.md#run-wholebodydynamics-on-an-external-pc).

- (OPTIONAL) type on a terminal `yarp rpc /wholeBodyDynamics/rpc` and execute the command `resetOffset all 300`. It will reset offsets of fake FT measurements, that might be affected by the results of a previous simulation. Fake FT measurements are used e.g. for defining the threshold for switching from single to double support balancing.
 
- Open the simulink model and run the module.

#### Citing this contribution
In case you want to cite the content of this module please refer to [iCub whole-body control through force regulation on rigid non-coplanar contacts](http://journal.frontiersin.org/article/10.3389/frobt.2015.00006/abstract) and use the following bibtex entry:

```
@INPROCEEDINGS{Nava_etal2016, 
author={G. Nava and F. Romano and F. Nori and D. Pucci}, 
booktitle={2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
title={Stability analysis and design of momentum-based controllers for humanoid robots}, 
year={2016}, 
pages={680-687}, 
keywords={Lyapunov methods;asymptotic stability;closed loop systems;control system synthesis;humanoid robots;legged locomotion;linearisation techniques;momentum;robust control;Lyapunov analysis;balancing controller design;closed loop system asymptotic stability;iCub humanoid robot;linearized system joint space;momentum-based controller design;robust controllers;stability analysis;unstable zero dynamics;walking controller design;Acceleration;Asymptotic stability;Humanoid robots;Legged locomotion;Robot kinematics;Stability analysis}, 
doi={10.1109/IROS.2016.7759126}, 
month={Oct},}
```

```
 @article{Nori_etal2015,
 author="Nori, F. and Traversaro, S. and Eljaik, J. and Romano, F. and Del Prete, A. and Pucci, D.",
 title="iCub whole-body control through force regulation on rigid non-coplanar contacts",
 year="2015",
 journal="Frontiers in {R}obotics and {A}{I}",
 volume="1"
 }
```
