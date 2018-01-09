# wholeBodyControllers

A collection of Matlab/Simulink whole body controllers for balancing and walking of humanoid robots. 

## Dependencies

This repository depends upon the following Software/repos:

- [Matlab/Simulink](https://it.mathworks.com/products/matlab.html), at least version **R2014a**.
- [WB-Toolbox](https://github.com/robotology/WB-Toolbox)
- [Gazebo Simulator](http://gazebosim.org/), at least version **7.8**
- [Gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins)
- [icub-gazebo](https://github.com/robotology/icub-gazebo) and [icub-gazebo-wholebody](https://github.com/robotology-playground/icub-gazebo-wholebody) for using iCub models.
- [codyco-modules](https://github.com/robotology/codyco-superbuild) (Optional, for [home positions]() and [wholeBodyDynamics]()).

It is suggested to install `codyco-modules`,`icub-gazebo`,`Gazebo-yarp-plugins` and `WB-Toolbox` using [codyco-superbuild](https://github.com/robotology/codyco-superbuild) (enable `CODYCO_USES_GAZEBO`, `CODYCO_USES_MATLAB`, options).

## Structure of the repo

- **config**: a collection of scripts used for correctly configure the repo. [[README]]()

- **controllers**: Simulink controllers for balancing and walking of humanoid robots. [[README]]()

- **doc**: guidelines on how to create/use Simulink models for control. [[README]]()

- **legacy**: legacy version of all Simulink models in the repo, written in the lowest supported matlab version (R2014a). [[README]]()

- **library**: a collection of functions/scripts in common between different controllers. [[README]]()

- **utilities**: Simulink models for debugging. [[README]]()

## Installation and usage

If all the required dependencies are installed, it is just necessary to clone this repository on your pc.

### Mantainers

Gabriele Nava (@gabrielenava)




