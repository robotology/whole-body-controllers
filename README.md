# whole-body-controllers

**Warning! This repository contains reseach material and therefore is under active development. In future releases, `master` branch may break compatibility with older versions of WBC. If you are interested in retrieving a `stable` version of this repo, `fork the repository` or refer to the following releases:**

- [WBC v2.0](https://github.com/robotology/whole-body-controllers/releases/tag/v2.0)
- [WBC v1.5](https://github.com/robotology/whole-body-controllers/releases/tag/v1.5)
- [WBC v1.0](https://github.com/robotology/whole-body-controllers/releases/tag/v1.0)

## Overview

The repository contains `Simulink-based whole-body controllers` developed to control the [iCub](http://www.icub.org/) humanoid robot. It can be imagined as a **starting point** and a **support** repository for a user that intends to develop a new Simulink controller (not necessarily for the iCub robot) in within the framework of the [robotology](https://github.com/robotology) organization. It is worth noting that:

- The controllers stored in this repository are an **overview** of the possibile control frameworks that can be implemented using the `robotology` software infrastructure. Also, the repository contains a [library](library/README.md) of configuration and utility Matlab functions to design simulations with [Gazebo](http://gazebosim.org/) simulator and on the real robot iCub. 

- The robot dynamics and kinematics is computed run-time by means of [WBToolbox](https://github.com/robotology/wb-toolbox), a Simulink library that wraps [iDyntree](https://github.com/robotology/idyntree). For more information on iDyntree library, see also this [README](https://github.com/robotology/idyntree/blob/master/README.md). 

- The Simulink models implement different control strategies both for fixed-base and for floating-base robots. They space from `momentum-based` torque control to `inverse-kinematics-based` position control. Have a look at the [controllers](controllers/README.md) folder for more details.

## Dependencies

This repository depends upon the following Software:

- [CMake](https://cmake.org/), at least version **3.5**.
- [Matlab/Simulink](https://it.mathworks.com/products/matlab.html), default version **R2017b**.
- [WB-Toolbox](https://github.com/robotology/WB-Toolbox) and [blockfactory](https://github.com/robotology/blockfactory).
- [Gazebo Simulator](http://gazebosim.org/), default version **9.0**.
- [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins).
- [icub-gazebo](https://github.com/robotology/icub-gazebo), [icub-gazebo-wholebody](https://github.com/robotology-playground/icub-gazebo-wholebody) and [icub-models](https://github.com/robotology/icub-models) to access iCub models.
- [whole-body-estimators](https://github.com/robotology/whole-body-estimators) (**Optional**, for using [wholeBodyDynamics](https://github.com/robotology/whole-body-estimators/tree/master/devices/wholeBodyDynamics) device).
- [YARP](https://github.com/robotology/yarp) and [icub-main](https://github.com/robotology/icub-main).

## Installation and usage

The repository is usually tested and developed on **Ubuntu** and **macOS** operating systems. Some functionalities may not work properly on **Windows**.

- **NOTE:** it is suggested to install `whole-body-controllers` and most of its dependencies (namely, `YARP`, `icub-main`, `whole-body-estimators`,`icub-gazebo`,`icub-gazebo-wholebody`, `icub-models`, `gazebo-yarp-plugins`, `blockfactory` and `WB-Toolbox` and their dependencies) using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) (enable `ROBOTOLOGY_USES_GAZEBO`, `ROBOTOLOGY_ENABLE_DYNAMICS`, `ROBOTOLOGY_USES_MATLAB` options).

- Otherwise, after installing all the dependencies, **clone the repository** on your pc by running on a terminal `git clone https://github.com/robotology/whole-body-controllers`, or download the repository. Then (on Ubuntu), open a terminal from the folder where you downloaded whole-body-controllers and run:
   
   ```
   mkdir build
   cd build
   ccmake ..
   ``` 
   in the GUI that it will open, set the `CMAKE_PREFIX_PATH` as your desired installation folder. Then, run `make install`.

- Set the environmental variable `YARP_ROBOT_NAME` in your `.bashrc` file (or equivalent) to be the name of the robot you want to control. List of supported robot names:

  | Robot Names | Associated URDF Model |
  |:-------------:|:-------------:|
  | iCubGenova02 | [model.urdf](https://github.com/robotology/icub-models/blob/master/iCub/robots/iCubGenova02/model.urdf) |
  | iCubGenova04 | [model.urdf](https://github.com/robotology/icub-models/blob/master/iCub/robots/iCubGenova04/model.urdf) |
  | iCubGazeboV2_5|[model.urdf](https://github.com/robotology/icub-models/blob/master/iCub/robots/iCubGazeboV2_5/model.urdf)|
  | icubGazeboSim |[model.urdf](https://github.com/robotology/yarp-wholebodyinterface/blob/master/app/robots/icubGazeboSim/model.urdf) |

- **IMPORTANT!** to use the WBC Simulink controllers, it is **required** to add the **installed** [+wbc](library/matlab-wbc/+wbc) folder to the Matlab path. There are two possible ways to add the folder to the Matlab path:
 
   **1a.** `manually` and `permanently` add the folder to the Matlab path;
   
   **1b.** run **only once** the [startup_WBC.m](config/startup_WBC.m.in) script, which is installed in your `${BUILD}` folder. In this case, path is **not** permanently added to Matlab, and it is required to **always** start Matlab from the folder where your `pathdef.m` file is (usually `~/Documents/MATLAB`). To facilitate the reaching of the WBC working folder from the folder containing the `pathdef.m`, a `goToWholeBodyController.m` script can be [automatically created](config/createGoToWBC.m) in that folder. Run it to jump to the WBC folder. For further information on the installation procedure see also the [WBToolbox documentation](https://robotology.github.io/wb-toolbox/mkdocs/install/#matlab).
    **WARNING**: if the repository is installed through the `robotology-superbuild`, **DO NOT** run the `startup_WBC.m` file but instead run the [startup_robotology_superbuild](https://github.com/robotology/robotology-superbuild/blob/master/cmake/template/startup_robotology_superbuild.m.in) file that comes along with robotology-superbuild installation.
   - **Note**: to use any function inside the package [matlab-wbc/+wbc](library/matlab-wbc/+wbc), add the `wbc` prefix to the function name when the function is invoked, i.e. `[outputs] = wbc.myFunction(inputs)`. More information on packages can be found in the [Matlab documentation](https://it.mathworks.com/help/matlab/matlab_oop/scoping-classes-with-packages.html).
   
- There are some functionalities of the repo such as the [automatic generation of c++ code from Simulink](https://github.com/robotology/whole-body-controllers#automatic-generation-of-c-code-from-simulink) that require to enable not-default cmake options. Check the available options by running `ccmake .` in your `build` directory.

## Troubleshooting

Please refer to the [WBToolbox troubleshooting documentation](https://robotology.github.io/wb-toolbox/mkdocs/troubleshooting/).

## Relevant folders of the repo

- **config**: a collection of scripts to correctly configure this repo. [[README]](config/README.md)

- **controllers**: Simulink whole-body position and torque controllers for balancing of humanoid robots. [[README]](controllers/README.md)

- **doc**: guidelines on how to create/use Simulink models for control. [[README]](doc/README.md)

- **library**: a library of functions/scripts used by the controllers. [[README]](library/README.md)

- **utilities**: Simulink models for debugging sensors on the real robot. [[README]](utilities/README.md)

### Available controllers 

- [fixed-base-joints-torque-control](controllers/fixed-base-joints-torque-control/README.md)
- [floating-base-balancing-position-control](controllers/floating-base-balancing-position-control/README.md)
- [floating-base-balancing-torque-control](controllers/floating-base-balancing-torque-control/README.md)
- [simulink-balancing-simulator](controllers/simulink-balancing-simulator/README.md)

### Matlab functions library

- [matlab WBC library](library/matlab-wbc)

## Additional features

### Automatic generation of c++ code from Simulink

There is the possibility to generate c++ code from the Simulink models using [Simulink coder](https://www.mathworks.com/products/simulink-coder.html) (**available only for the [floating-base-balancing-torque-control](controllers/floating-base-balancing-torque-control)**). The repositiory that contains the generated c++ code is named [autogenerated-whole-body-controllers](https://github.com/robotology-playground/autogenerated-whole-body-controllers). Documentation on how to generate the code is available in the repository [wiki](https://github.com/robotology-playground/autogenerated-whole-body-controllers/wiki/How-to-generate-code-from-a-Simulink-model).

### Static GUI for Simulink

When used for controlling real platforms, heavy Simulink models may violate the user-defined simulation time step, see also [this issue](https://github.com/robotology/wb-toolbox/issues/160). It seems a source of delay is the run-time update of the Simulink interface. For this reason, a [static GUI for running the models](library/matlab-gui) has been developed. If you want to run Simulink with the static GUI, run the [startModelWithStaticGui](controllers/floating-base-balancing-torque-control/startModelWithStaticGui.m) script.

### Home positions for yarpmotorgui

The repo contains a set of predefined [home positions](utilities/homePositions) to be used with the [yarpmotorgui](https://www.yarp.it/yarpmotorgui.html). By default, if the repo is installed through `robotology-superbuild`, the home positions are installed in the `robotology-superbuild/build/install` directory. Otherwise add the path to the [homePositions](utilities/homePositions) folder to the `YARP_DATA_DIRS` environmental variable in your `.bashrc` file . The command to use the home positions with the yarpmotorgui is `yarpmotorgui --from myHomePosFileName.ini`. 

## Where do I find legacy materials?

Official legacy repositories are: [mex-wholebodymodel](https://github.com/robotology/mex-wholebodymodel) and [WBI-Toolbox-controllers](https://github.com/robotology-legacy/WBI-Toolbox-controllers). **Note**: these legacy repos contain undocumented/outdated code, and duplicated or not tested matlab functions. They also contain original code that has been tested on the robot in the past and then never used again, or code that will be ported in the main repository in the future.

- [exploit friction controller](https://github.com/robotology-legacy/WBI-Toolbox-controllers/tree/master/controllers/momentum-based-yoga-friction)
- [walking controller](https://github.com/robotology-legacy/WBI-Toolbox-controllers/tree/master/controllers/task-based-walking)
- [seesaw controller](https://github.com/robotology-legacy/WBI-Toolbox-controllers/tree/master/controllers/torqueBalancingOnSeesaw)
- [automatic gain tuning](https://github.com/robotology-legacy/WBI-Toolbox-controllers/tree/master/controllers/torqueBalancingTuning) and [automatic gain tuning-matlab](https://github.com/robotology/mex-wholebodymodel/tree/master/controllers/torqueBalancingGainTuning)
- [elastic joints control](https://github.com/robotology/mex-wholebodymodel/tree/master/controllers/torqueBalancing_JE)
- [walkman control](https://github.com/robotology-legacy/WBI-Toolbox-controllers/tree/master/controllers/torqueBalancing-walkman) and [walkman control-matlab](https://github.com/robotology/mex-wholebodymodel/tree/master/controllers/torqueBalancingWalkman)
- [joint-space control and centroidal transformation](https://github.com/robotology/mex-wholebodymodel/tree/master/controllers/torqueBalancingJointControl)
- [stand-up control 4 contacts](https://github.com/robotology-legacy/WBI-Toolbox-controllers/tree/master/controllers/torqueBalancingStandup_4Contacts)

## Citing this work

If you are using this code for your research activity and you're willing to cite it, you may add the following references to your bibliography:

```
  @INPROCEEDINGS{Nava_etal2016,
  author={G. Nava and F. Romano and F. Nori and D. Pucci}, 
  booktitle={2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Stability analysis and design of momentum-based controllers for humanoid robots}, 
  year={2016}, 
  pages={680-687}, 
  doi={10.1109/IROS.2016.7759126}, 
  month={Oct},
  }
```

```
  @article{Nori_etal2015,
  author="Nori, F. and Traversaro, S. and Eljaik, J. and Romano, F. and Del Prete, A. and Pucci, D.",
  title="iCub whole-body control through force regulation on rigid non-coplanar contacts",
  year="2015",
  journal="Frontiers in {R}obotics and {A}{I}",
  volume="1",
  }
```

## Mantainers

Gabriele Nava ([@gabrielenava](https://github.com/gabrielenava))
