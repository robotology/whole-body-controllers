# Simulink library for whole-body-controllers

It is a library of Simulink blocks (and the associated MATLAB functions) used in whole-body-controllers.

## Overview

The library will appear in the Simulink Library Browser with the name specified inside [slblocks.m](slblocks.m). The main library folder is divided into subfolders. Each subfolder has a main model -- see, e.g., [MomentumVelocityControl](MomentumVelocityControl/momentumVelocityControl_lib.slx). Each library element in the subfolders is contained in the subfolder main model. Then, each subfolder main model is linked to `WBC_lib_main.slx`;

## Installation and usage

The models are installed in the `$CMAKE_INSTALL_PREFIX/mex/+wbc/simulink` folder. To link the library to the Simulink Library Browser, after installation, from MATLAB run only once the [startup_WBC.m](config/startup_WBC.m.in) script, which is installed in your `${BUILD}` folder. Path is **not** permanently added to Matlab, and it is required to **always** start Matlab from the folder where your `pathdef.m` file is (usually `~/Documents/MATLAB`). If the repository is installed through the `robotology-superbuild`, **DO NOT** run the `startup_WBC.m` file but instead run the [startup_robotology_superbuild](https://github.com/robotology/robotology-superbuild/blob/master/cmake/template/startup_robotology_superbuild.m.in) file that comes along with robotology-superbuild installation.

To use the MATLAB functions that are installed with the library, the function name must be preceded by the namespace `wbc`. E.g. `[] = wbc.myLibraryFunction()`.

## How to link new elements to the (sub)libraries

- open the (sub)library (e.g., `WBC_lib_main.slx`);

- create a new subsystem and remove in/out elements;

- in the subsystem's property (right click on it to access), select `callbacks/openfcn`;

- add the name of the sublibrary model to be linked by the subsystem;

Now `WBC_lib_main.slx` links to your library model as a sublib in the Simulink Library Browser.

## List of available sublibraries

- [MomentumVelocityControl](MomentumVelocityControl) 

- [Utilities](Utilities) 



