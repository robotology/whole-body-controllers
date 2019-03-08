# Configuration scripts

A collection of scripts used for configuring the repo.

- [export_WBC.m](export_WBC.m): run this script. Then, digit the Matlab version in which you want to export the Simulink models. All models in the repo will be exported to that version. By default, Simulnk models in this repo are written using Matlab 2017b. **Remember: you cannot export a model in a Matlab version newer than the one you are using**! 

- [startup_WBC.m](startup_WBC.m): run this script. Then, the path to the `matlab-wbc` folder will be **permanently** added to your `pathdef.m` file, which will be saved inside the Matlab `userpath`. In order to have the `matlab-wbc` folder inside the Matlab path, it is **required** to start Matlab from the folder where the `pathdef.m` file is (i.e., from the folder that the `userpath` is pointing, usually `~/Documents/MATLAB`).