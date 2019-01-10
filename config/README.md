# Configuration scripts

A collection of scripts used for correctly configure the repo.

- [export_WBC.m](export_WBC.m): run this script once. Then, digit the Matlab version in which you want to export the Simulink models. All models in the repo will be exported in that version. By default, Simulnk models in this repo are written using Matlab 2017b. **Remember: you cannot export a model in a Matlab version newer than the one you are using**! 

- [startup.m](startup.m): run this script once. Then, the path to the `matlab-wbc` folder will be permanently added to the `pathdef.m` file, which is saved inside the Matlab `userpath`. In order to have the `matlab-wbc` folder inside the Matlab path, it is required to start Matlab from the folder where the `pathdef.m` file is (in general, `~/Documents/MATLAB`).





