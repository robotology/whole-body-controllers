# whole-body-controllers

A collection of Matlab/Simulink whole body controllers for balancing and walking of humanoid robots. 

## Dependencies

This repository depends upon the following Software/repositories:

- [Matlab/Simulink](https://it.mathworks.com/products/matlab.html), at least version **R2014a**
- [WB-Toolbox](https://github.com/robotology/WB-Toolbox)
- [Gazebo Simulator](http://gazebosim.org/), at least version **7.8**
- [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins)
- [icub-gazebo](https://github.com/robotology/icub-gazebo) and [icub-gazebo-wholebody](https://github.com/robotology-playground/icub-gazebo-wholebody) to access iCub models.
- [codyco-modules](https://github.com/robotology/codyco-modules) (Optional, for using [home positions](https://github.com/robotology/codyco-modules/tree/master/src/modules/torqueBalancing/app/robots) and [wholeBodyDynamics](https://github.com/robotology/codyco-modules/tree/master/src/devices/wholeBodyDynamics) device).

**NOTE:** it is suggested to install `codyco-modules`,`icub-gazebo`,`icub-gazebo-wholebody`, `gazebo-yarp-plugins` and `WB-Toolbox` (and their dependencies) using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) (enable `ROBOTOLOGY_USES_GAZEBO`, `ROBOTOLOGY_ENABLE_DYNAMICS`, `ROBOTOLOGY_USES_MATLAB` options).

## Structure of the repo

- **config**: a collection of scripts for correctly configure this repo. [[README]](config/README.md)

- **torque-controllers**: Simulink torque controllers for balancing and walking of humanoid robots. [[README]](torque-controllers/README.md)

- **position-controllers**: Simulink position controllers for balancing and walking of humanoid robots. [[README]](position-controllers/README.md)

- **doc**: guidelines on how to create/use Simulink models for control. [[README]](doc/README.md)

- **legacy**: legacy version of all Simulink models in the repo, written in the lowest supported matlab version (R2014a). [[README]](legacy/README.md)

- **library**: an internal library of functions/scripts used by the controllers. [[README]](library/README.md)

- **utilities**: Simulink models for debugging sensors on the real robot. [[README]](utilities/README.md)

## Installation and usage

If all the required dependencies are correctly installed and configured, it is just necessary to clone this repository on your pc. To use the controllers, follow the [documentation](doc/README.md) and the controllers READMEs.

### Mantainers

Gabriele Nava ([@gabrielenava](https://github.com/gabrielenava))




