# Library

## matlab-wbc

A package of utility Matlab functions used by all the controllers.

### How to install the folder

The `+wbc` folder is installed in your `${INSTALL/mex}` directory. The **installed** folder must be added to the Matlab path. This can be done by setting the path manually or by running the `startup_WBC.m` script which is also installed in your `${BUILD}` folder. If you chose to run the `startup_WBC.m` script, remember to **always** start matlab from the folder where the `pathdef.m` file is (usually `~/Documents/MATLAB`).

### How to use the +wbc library

To use any function inside the package [matlab-wbc/+wbc](matlab-wbc/+wbc), add the `wbc` prefix to the function name when the function is invoked, i.e. `[outputs] = wbc.myFunction(inputs)`. More information can be found in the [Matlab documentation](https://it.mathworks.com/help/matlab/matlab_oop/scoping-classes-with-packages.html).

## matlab-gui

Utilties for designing a `static GUI` for starting/closing the Simulink models without the need of opening the Simulink interface.

## simulink-library

A library of Simulink models for computing specific quantities required by the controllers. **Usage**: all models are exposed in the Simulink library browser. Look for the block you need and drag and drop it in your Simulink model.
