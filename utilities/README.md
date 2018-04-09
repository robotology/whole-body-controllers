# Utilities

Simulink models for debugging. 

- `debug_FTMeas.mdl`: This model is used for checking the raw measurements coming from iCub legs and feet FT sensors. **USAGE:** this model is supposed to be used only with `iCubGenova02` and `iCubGenova04` robots. 

- `debug_FTMeas_shoes.mdl`: This model is used for checking the raw measurements coming from the sensorized shoes FT sensors. **USAGE:** this model is supposed to be used only with the `sensorized shoes`. 

- `debug_BoschIMU.mdl`: This model is used for checking if the iCub Bosch IMU (located in the iCub head) is correclty aligned with the gravity. **USAGE:** this model is supposed to be used only with `iCubGenova02` and `iCubGenova04` robots, and only with the robot on the pole.

- `debug_xSensIMU.mdl`: This model is used for checking if the iCub xSens IMU (located in the iCub root link) is correclty aligned with the gravity. **USAGE:** this model is supposed to be used only with `iCubGenova04` robot.

- `debug_seesawIMU.mdl`: This model is used for checking if the seesaw IMU (located in the seesaw board) is correclty aligned with the gravity. **USAGE:** this model is supposed to be used only with the `seesaw board`.

- `releaseLegStressesWhileStanding.sh`: Run this script to release the internal stresses in the robot legs while standing on two feet. **USAGE:** this script is supposed to be used only with `iCubGenova02` and `iCubGenova04`. Run on a terminal `cd PATH/TO/THIS/FOLDER && sh releaseLegStressesWhileStanding.sh`.

