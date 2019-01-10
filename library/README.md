# Library

## matlab-wbc

A collection of utlity functions/scripts used by the controllers.

### How to install the folder

Add the `matlab-wbc` folder to the Matlab path, or run the `startup.m` script in the `config` folder. If you chose to run the `startup.m` script, remember to start matlab from the folder where the `pathdef.m` file is (usually `~/Documents/MATLAB`).

### How to use the +wbc library

To use any function inside the package [matlab-wbc/+wbc](matlab-wbc/+wbc), add the `wbc` prefix to the function name when the function is invoked, i.e. `[outputs] = wbc.myFunction(inputs)`. More information can be found in the [Matlab documentation](https://it.mathworks.com/help/matlab/matlab_oop/scoping-classes-with-packages.html).

## matlab-gui

A static GUI for starting/closing the Simulink models without the need of opening them.


