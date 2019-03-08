## HOW TO RUN A SIMULATION WITH TORQUE CONTROL ON ICUB

The procedure to run the torque balancing module is still quite elaborate. Users willing to use the module should follow this list.

- Set the environmental variable YARP_ROBOT_NAME in the `.bashrc` according to the robot one wants to use (e.g. `iCubGazeboV2_5` or `icubGazeboSim` for simulations).

- Verify that Gazebo and the robot model for simulations are available and installed. You can check if the controller is targeting the correct robot model by typing on a terminal:

```
yarp resource --find model.urdf

```

  then, check that the path and the model name are correct.

- Launch the `yarpserver` (with `--write` option if necessary).

- Launch gazebo. It is in general required to use the synchronization between the controller and the simulator to avoid real-time factor related problems. Therefore launch gazebo as follows: `gazebo -slibgazebo_yarp_clock.so`.
 
- Bring the robot in a suitable home position (e.g. use the `$ yarpmotorgui --from homePoseBalancing.ini` and then select a custom position by clicking on `Global Joints Commands/Custom postions`).

- For `icubGazeboSim` robot, launch `wholeBodyDynamics` as follows: `YARP_ROBOT_NAME=icubGazeboSim yarprobotinterface --config launch-wholebodydynamics.xml`. Same holds for `iCubGazeboV2_5` (just change the robot name). For further details see [here](https://github.com/robotology/codyco-modules/blob/master/doc/force_control_on_icub.md#run-wholebodydynamics-on-an-external-pc).

- (OPTIONAL) type on a terminal `yarp rpc /wholeBodyDynamics/rpc` and execute the command `resetOffset all 300`. It will reset offsets of the fake FT measurements, that might be affected by the results of a previous simulation. Fake FT measurements are used e.g. for defining the threshold for switching from single to double support balancing.
 
- Open the simulink model and run the module.