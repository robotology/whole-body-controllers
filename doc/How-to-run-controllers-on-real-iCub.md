## HOW TO RUN BALANCING WITH SIMULINK CONTROLLERS ON ICUB

#### Preliminary procedures:

- Set the environmental variable `YARP_ROBOT_NAME` in the `.bashrc` according to the robot one wants to use (e.g. `iCubGenova04`, etc. for experiments).
 
- Start the robot. Please refer to [How to setup iCub for whole-body control experiments](How-to-setup-iCub-for-wbc-experiments.md) for more information on the startup procedure.

#### Before putting the robot feet on the ground:

- Bring the robot in a suitable home position. Run on a terminal the command `yarpmotorgui --from homePoseBalancing.ini` and then select a custom position by clicking on the window `Global Joints Commands/Custom postions`.

- Type on a terminal the command `yarp rpc /wholeBodyDynamics/rpc` and then execute the command `calib all 300`. It will remove offsets from FT sensors measurements.

- Then, put the robot on the ground.

#### After putting the robot on the ground:

- Open the simulink model and run the controller.
