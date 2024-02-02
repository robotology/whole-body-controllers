# Configuration scripts

A collection of scripts used for configuring the repo.

- [exportWBCto2019b.m](exportWBCto2019b.m): export all models in the repo to MATLAB 2019b. Necessary to ensure compatibility of the models up to the oldest supported MATLAB version. **Remember: you cannot export a model in a Matlab version newer than the one you are using**! 

- [startup_WBC.m.in](startup_WBC.m.in): **DO NOT** run this script, but run its installation inside the procjet `${BUILD}` folder. Then, the path to the `+wbc` folder will be **permanently** added to your `pathdef.m` file, which will be saved inside the Matlab `userpath`. **WARNING**! In order to have the `+wbc` folder inside the Matlab path, it is **required** to start Matlab from the folder where the `pathdef.m` file is (i.e., from the folder that the `userpath` is pointing, usually `~/Documents/MATLAB`).

- [createGoToWBC.m](createGoToWBC.m): run this script. A file named 'goToWholeBodyControllers.m' will be created in the folder pointed by your `userpath`, to facilitate the reaching of WBC source folder when Matlab is started from the `userpath` folder.
