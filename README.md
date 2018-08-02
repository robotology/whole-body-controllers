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

**NOTE:** it is suggested to install `whole-body-controllers` and most of its dependencies (namely, `codyco-modules`,`icub-gazebo`,`icub-gazebo-wholebody`, `gazebo-yarp-plugins` and `WB-Toolbox` and their dependencies) using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) (enable `ROBOTOLOGY_USES_GAZEBO`, `ROBOTOLOGY_ENABLE_DYNAMICS`, `ROBOTOLOGY_USES_MATLAB` options).

## Installation and usage

- set the environmental variable `YARP_ROBOT_NAME` in the `.bashrc` file to be the name of the robot you want to control. List of supported robot names:

| Robot Names | 
|:-------------:|
| iCubGenova02 |  
| iCubGenova04 | 
| iCubGazeboV2_5 |
| icubGazeboSim |

- if all the required dependencies are correctly installed and configured, just clone this repository on your pc. No further actions are required. An alternative is to install this repository through the `robotology-superbuild`. It is required to enable the option `ROBOTOLOGY_ENABLE_DYNAMICS`. **Important:** it is necessary to set the 

## Structure of the repo

- **config**: a collection of scripts for correctly configure this repo. [[README]](config/README.md)

- **torque-controllers**: Simulink torque controllers for balancing and walking of humanoid robots. [[README]](torque-controllers/README.md)

- **doc**: guidelines on how to create/use Simulink models for control. [[README]](doc/README.md)

- **legacy**: legacy version of all Simulink models in the repo, written in the lowest supported matlab version (R2014a). [[README]](legacy/README.md)

- **library**: a library of functions/scripts used by the controllers. [[README]](library/README.md)

- **utilities**: Simulink models for debugging sensors on the real robot. [[README]](utilities/README.md)

#### Where do I find new features and legacy repos?

##### Available controllers 

- [impedance-control](https://github.com/robotology/whole-body-controllers/tree/master/torque-controllers/impedance-control)
- [momentum-based-standup](https://github.com/robotology/whole-body-controllers/tree/master/torque-controllers/momentum-based-standup)
- [momentum-based-yoga](https://github.com/robotology/whole-body-controllers/tree/master/torque-controllers/momentum-based-yoga)
- [utilities](https://github.com/robotology/whole-body-controllers/tree/master/utilities)

##### Matlab functions library

- [matlab library](https://github.com/robotology/whole-body-controllers/tree/master/library/matlab)

##### Active Forks (new features)

- [force-parametrization](https://github.com/ahmadgazar/whole-body-controllers)
- [PhRI-standup](https://github.com/Yeshasvitvs/wholeBodyControllers)
- [exploit friction and walking controller](https://github.com/gabrielenava/whole-body-controllers)

##### Legacy

- [seesaw controller and integration-based-ikin](https://github.com/gabrielenava/whole-body-controllers/tree/legacy)
- [automatic gain tuning](https://github.com/gabrielenava/mex-wholebodymodel/tree/master/controllers/torqueBalancingGainTuning)
- [elastic joints control](https://github.com/gabrielenava/mex-wholebodymodel/tree/master/controllers/torqueBalancing_JE)
- Walkman control: https://github.com/gabrielenava/mex-wholebodymodel/tree/master/controllers/torqueBalancingWalkman and https://github.com/gabrielenava/WBI-Toolbox-controllers/tree/walkman
- [joint-space control and centroidal transformation](https://github.com/gabrielenava/mex-wholebodymodel/tree/master/controllers/torqueBalancingJointControl)

#### Associated repositories

- [idyntree-high-level-wrappers](https://github.com/gabrielenava/idyntree-high-level-wrappers)

### Mantainers

Gabriele Nava ([@gabrielenava](https://github.com/gabrielenava))
