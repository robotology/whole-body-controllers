## HOW TO SETUP ICUB FOR WHOLE BODY CONTROL EXPERIMENTS

**Disclaimer:** this documentation is not a complete guide on how to work with iCub, and it is intended to be used by users that already have an idea of what they are doing. If you are looking for a more detailed guide, check the [icub-wiki](http://wiki.icub.org/wiki/Main_Page).

### Preliminary checks

#### Update yarp, icub-main and robotology-superbuild

First, it is necessary to update the `yarp`, `icub-main` and `robotology-superbuild` directories on the iCub laptop, as well as on the computer used for launching the Simulink controllers (if different). You can do this by opening a terminal, and by pointing the **source folder** of each repository (e.g. the `yarp` folder). Then, type the command:

`git pull` 

You don't have to repeat this operation on computers which share sources folders with the iCub laptop.

Then, if there are some updates, you can compile them by running the commands: 

`cd build` 
`cmake  .` 
`make` 

You may also need to update all the repositories downloaded through the `robotology-superbuild`. Go into the `robotology-superbuild` folder and run:

`cd build` 
`make update-all` 

**IMPORTANT** before running `make-update all`, be sure all repositories downloaded with the superbuild are in `master` branch (repositories are inside the `robotology-superbuild/robotology` folder. Go into the source directory of each repo (e.g. `iDyntree`) and check the current github branch by running the `git status` command. If they are not in `master` branch, then be sure the `YCM_DEVEL_MODE` option is activated for the corresponding repository. You can check this in the `robotology-superbuild/build` folder, by running:

`ccmake ./`

and by looking at the status of the options of the format `YCM_DEVEL_MODE_name_of_the_repo`.

To update the iCub on-board PC (`pc104` or `icub-head`), first connect to it using the command 

`ssh pc104` 
or 
`ssh icub-head` 

and if necessary add -X option to redirect the graphic output to your local machine. Then follow the same procedure presented before.

### Firmware update

**iCub software version anterior to 1.8.0**: 

- It is not recommended to perform the firmware update without the support from IIT.

**Latest iCub software version**: 

- Look [here](https://github.com/robotology/QA/issues/240) for instructions on how to perform an update using `Firmware-updater`.

- You can find information about CAN-bus numbers and CAN-bus device drivers [here](http://wiki.icub.org/wiki/Can_addresses_and_associated_firmware#Can_Networks). 

### Calibration of the robot 

#### Joint encoders calibration

On iCub it is sometimes required to re-calibrate the “zero position” associated to the joints of the robot. The robot should be fixed on the pole for calibration. Fine calibration can be done with the help of a level tool, as described in the [wiki](http://wiki.icub.org/wiki/Manual#Three._Calibration). Remember to calibrate the `torso` joints before the `neck` and `arms` joints. For example:

- On a terminal run the  `yarpmotorgui`, then switch to the tab associated to the body part you want to calibrate. Then, click the `idle` button for the joints to calibrate, allowing you to move them freely. With the help of a level tool, move the joint such that you can measure that it is level. Read the joint encoder value corresponding to the "level" position. 

- This value will be added in the specific file associated to the joint. On the `on-board PC`, go to the folder containing the "\.xml" calibration files for your robot and open the corresponding file (example: `.../robots/$ROBOT_NAME/calibrators/left\_leg\_calib.xml`). Add the encoder value that you have previously noted to the current `calibrationDelta` of the joint (each number of the parameter `calibrationDelta` corresponds to a joint). For example, we add some values to the line: 

  ```
  <param name="calibrationDelta"> -5.0  8.7  -11.4  -0.6  0.0 </param> 
  ```
Restart the robot to apply the modifications. Check that the joints were successfully calibrated.

#### IMU calibration

For IMU calibration, the robot has to be on the pole, in home position (i.e. all joints to 0). Actually, we calibrate the neck `pitch` and `roll` such that the IMU `linear accelerations` readings are, as expected, [0 0 9.81] along the [x, y, z] axis of an inertial reference frame with the `z` axis pointing against the gravity.

You need to receive the acceleration values of the head to be able to calibrate it: you may launch (or create) a program which reads the IMU acceleration data. Just for that, this repository provides the Simulink model [debug_BoschIMU.mdl](https://github.com/robotology/whole-body-controllers/blob/master/utilities/debug_BoschIMU.mdl).

In `yarpmotorgui`, switch to the tab associated to the head. Then, click the `idle` button for the joints to calibrate: `neck_pitch` and `neck_roll`, allowing you to move them freely. Then, move the head joints until signals are near their desired values. Note the encoder values and proceed as before. 

### Launch the iCub 

Once everything is correctly updated and calibrated:

1. Switch on the power supplies (wait a little for the power to come to the iCub). 
 
2. Launch `yarpmanager` on the icub external PC. Properly configure the `cluster`. Then run the application `icub_startup_wbd.xml`. 
 
### Some little things to know 
 
 - Be careful with the head and hands of the robot. 

 - How to hide DEBUG information: in the cmake information, put the variable DEBUG from DEBUG to RELEASE. 
