# Utilities

Simulink models for sensors debugging and other utilities. 

- [homePositions](homePositions/README.md): home positions of the iCub robots to be used with the `yarpmotorgui`.
- `debug_FTExternalForces.mdl`:  this model is used for checking the measurements coming from iCub legs, arms and feet FT sensors. **USAGE:** this model is supposed to be used only with `iCubGenova02` and `iCubGenova04` robots. 
- `debug_FTMeas.mdl`: this model is used for checking the raw measurements coming from iCub legs, arms and feet FT sensors. **USAGE:** this model is supposed to be used only with `iCubGenova02` and `iCubGenova04` robots. 
- `debug_FTMeas_shoes.mdl`: this model is used for checking the raw measurements coming from the sensorized shoes FT sensors. **USAGE:** this model is supposed to be used only with the `sensorized shoes`. 
- `debug_positionControl.mdl`: this model is used for checking the joint errors when in position control. It needs a series of ``controlBoardDumper``s running (check the comment inside the ``mdl`` itself).
- `debug_BoschIMU.mdl`: this model is used for checking if the iCub Bosch IMU (located in the iCub head) is correclty aligned with the gravity. **USAGE:** this model is supposed to be used only with `iCubGenova02` and `iCubGenova04` robots, and only with the robot on the pole.
- `debug_xSensIMU.mdl`: this model is used for checking if the iCub xSens IMU (located in the iCub root link) is correclty aligned with the gravity. **USAGE:** this model is supposed to be used only with `iCubGenova04` robot on the pole.
- `debug_seesawIMU.mdl`: this model is used for checking if the seesaw IMU (located in the seesaw board) is correclty aligned with the gravity. **USAGE:** this model is supposed to be used only with the `seesaw board`.
- `releaseLegStressesWhileStanding.sh`: run this script to release the internal stresses in the robot legs while standing on two feet. **USAGE:** this script is supposed to be used only with `iCubGenova02` and `iCubGenova04` robots. Run on a terminal `cd PATH/TO/THIS/FOLDER && sh releaseLegStressesWhileStanding.sh`.
- ``plotRobotFrames.m`` allows displaying a set of frames given the robot model.
