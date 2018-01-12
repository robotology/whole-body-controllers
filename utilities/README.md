# Utilities

Simulink models for debugging. 

- debug_FTMeas.mdl: This model is used for checking the raw measurements coming from iCub legs and feet FT sensors. **USAGE:** this model is supposed to be used only with `iCubGenova02` and `iCubGenova04` robots. 

-  debug_IMUCalib.mdl: This model is used for checking if the IMU is correclty aligned with gravity. **USAGE:** this model is supposed to be used only with `iCubGenova02` and `iCubGenova04` robots, and oly with the robot on the pole.

- releaseLegStressesWhileStanding.sh: Run this script to release the internal stresses in the robot legs while standing on two feet. **USAGE:** this script is supposed to be used only with `iCubGenova02` and `iCubGenova04`. Run on a terminal `cd $THIS FOLDER && sh releaseLegStressesWhileStanding.sh`.



