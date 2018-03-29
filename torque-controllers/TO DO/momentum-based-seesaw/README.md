# iCub on a seesaw

This controller aims to balance the humanoid robot iCub while standing on a moving platform, i.e. the seesaw board.

## how to run a simulation:

- install [icub-gazebo-wholebody](https://github.com/robotology-playground/icub-gazebo-wholebody) repository and follow the [seesaw README](https://github.com/robotology-playground/icub-gazebo-wholebody/blob/master/worlds/icub_seesaw_world/README.md) instruction for opening the model in Gazebo.
- run `yarpserver` on a terminal.
- run Gazebo on another terminal by using the command you can find [here](https://github.com/robotology-playground/icub-gazebo-wholebody/blob/master/worlds/icub_seesaw_world/README.md).
- it is also necessary to run the estimator thread. In simulation, open a terminal and type `YARP_ROBOT_NAME=<name of your robot> yarprobotinterface --config launch-wholebodydynamics.xml`. Further information can be found [here](https://github.com/robotology/codyco-modules/blob/master/doc/force_control_on_icub.md#run-wholebodydynamics-on-an-external-pc).
- set your simulation preferences in the [initialization file](initTrqBalancingSeeSaw.m) (in particular, select the proper `YARP_ROBOT_NAME` according to the robot you are using. 
- open the [simulink model](controller.mdl) and let it run!

