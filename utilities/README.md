# Utilities

Simulink models for sensors debugging and other utilities. 

- [homePositions](homePositions/README.md): home positions of the iCub robots to be used with the `yarpmotorgui`.
- `debug_FTExternalForces.mdl`:  this model is used for checking the measurements coming from iCub legs, arms and feet FT sensors. **USAGE:** this model is supposed to be used only with `iCubGenova02` and `iCubGenova04` robots. 
- `debug_FTMeas.mdl`: this model is used for checking the raw measurements coming from iCub legs, arms and feet FT sensors. **USAGE:** this model is supposed to be used only with `iCubGenova02` and `iCubGenova04` robots. 
- `debug_FTMeas_shoes.mdl`: this model is used for checking the raw measurements coming from the sensorized shoes FT sensors. **USAGE:** this model is supposed to be used only with the `sensorized shoes`. 
- `debug_positionControl.mdl`: this model is used for checking the joint errors when in position control. It needs a series of ``controlBoardDumper``s running (check the comment inside the ``mdl`` itself).
- `releaseLegStressesWhileStanding.sh`: run this script to release the internal stresses in the robot legs while standing on two feet. **USAGE:** this script is supposed to be used only with `iCubGenova02` and `iCubGenova04` robots. Run on a terminal `cd PATH/TO/THIS/FOLDER && sh releaseLegStressesWhileStanding.sh`.
- ``plotRobotFrames.m`` allows displaying a set of frames given the robot model.
