# whole-body-controllers

The repository contains Simulink-based whole-body controllers for humanoid robots, and configuration and utility Matlab functions to perform balancing simulations with Gazebo simulator and on the Yarp-based robot [iCub](http://www.icub.org/). 

**Warning! This repository is under active development. In future releases, `master` branch may break compatibility with older versions of WBC. If you are interested in retrieving a `stable` version of this repo, `fork the repository` or refer to the following releases:**

- [WBC v2.0](https://github.com/robotology/whole-body-controllers/releases/tag/v2.0)
- [WBC v1.5](https://github.com/robotology/whole-body-controllers/releases/tag/v1.5)
- [WBC v1.0](https://github.com/robotology/whole-body-controllers/releases/tag/v1.0)

## Dependencies

This repository depends upon the following Software:

- [Matlab/Simulink](https://it.mathworks.com/products/matlab.html), default version **R2017b**
- [WB-Toolbox](https://github.com/robotology/WB-Toolbox) and [blockfactory](https://github.com/robotology/blockfactory)
- [Gazebo Simulator](http://gazebosim.org/), default version **9.0**
- [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins)
- [icub-gazebo](https://github.com/robotology/icub-gazebo), [icub-gazebo-wholebody](https://github.com/robotology-playground/icub-gazebo-wholebody) and [icub-models](https://github.com/robotology/icub-models) to access iCub models.
- [codyco-modules](https://github.com/robotology/codyco-modules) (Optional, for using [home positions](https://github.com/robotology/codyco-modules/tree/master/src/modules/torqueBalancing/app/robots) and [wholeBodyDynamics](https://github.com/robotology/codyco-modules/tree/master/src/devices/wholeBodyDynamics) device).

## Installation and usage

**NOTE:** it is suggested to install `whole-body-controllers` and most of its dependencies (namely, `codyco-modules`,`icub-gazebo`,`icub-gazebo-wholebody`, `icub-models`, `gazebo-yarp-plugins`, `blockfactory` and `WB-Toolbox` and their dependencies) using the [robotology-superbuild](https://github.com/robotology/robotology-superbuild) (enable `ROBOTOLOGY_USES_GAZEBO`, `ROBOTOLOGY_ENABLE_DYNAMICS`, `ROBOTOLOGY_USES_MATLAB` options).

- Otherwise, clone the repository on your pc by running on a terminal `git clone https://github.com/robotology/whole-body-controllers`, or dowload the repository.

- set the environmental variable `YARP_ROBOT_NAME` in the `.bashrc` file to be the name of the robot you want to control. List of supported robot names:

  | Robot Names | Associated URDF Model |
  |:-------------:|:-------------:|
  | iCubGenova02 | [model.urdf](https://github.com/robotology/icub-models/blob/master/iCub/robots/iCubGenova02/model.urdf) |
  | iCubGenova04 | [model.urdf](https://github.com/robotology/icub-models/blob/master/iCub/robots/iCubGenova04/model.urdf) |
  | iCubGazeboV2_5|[model.urdf](https://github.com/robotology/icub-models/blob/master/iCub/robots/iCubGazeboV2_5/model.urdf)|
  | icubGazeboSim |[model.urdf](https://github.com/robotology/yarp-wholebodyinterface/blob/master/app/robots/icubGazeboSim/model.urdf) |

- to use the Simulink controllers, it is **required** to add the [matlab-wbc](library/matlab-wbc) folder to the Matlab path. There are two different possible ways to add the folder to the path: 
   - manually add the folder to the Matlab path;
   - run **only once** the [startup_WBC.m](config/startup_WBC.m) script. In this case, it is required to **always** start matlab from the folder where the `pathdef.m` file is (usually `~/Documents/MATLAB`). For further information see also the [WBToolbox documentation](https://robotology.github.io/wb-toolbox/mkdocs/install/#matlab).
   
   **Note**: to use any function inside the package [matlab-wbc/+wbc](library/matlab-wbc/+wbc), add the `wbc` prefix to the function name when the function is invoked, i.e. 
   
   `[outputs] = wbc.myFunction(inputs)`. 
   
   More information on packages can be found in the [Matlab documentation](https://it.mathworks.com/help/matlab/matlab_oop/scoping-classes-with-packages.html).

## Troubleshooting

Please refer to the [WBToolbox troubleshooting documentation](https://robotology.github.io/wb-toolbox/mkdocs/troubleshooting/).

## Structure of the repo

- **config**: a collection of scripts for correctly configure this repo. [[README]](config/README.md)

- **controllers**: Simulink whole-body position and torque controllers for balancing of humanoid robots. [[README]](controllers/README.md)

- **doc**: guidelines on how to create/use Simulink models for control. [[README]](doc/README.md)

- **library**: a library of functions/scripts used by the controllers. [[README]](library/README.md)

- **utilities**: Simulink models for debugging sensors on the real robot. [[README]](utilities/README.md)

### Available controllers 

- [fixed-base-joints-control](controllers/fixed-base-joints-control/README.md)
- [floating-base-balancing-position-control](controllers/floating-base-balancing-position-control/README.md)
- [floating-base-balancing-torque-control](controllers/floating-base-balancing-torque-control/README.md)

### Matlab functions library

- [matlab WBC library](library/matlab-wbc)

## Additional features

### Automatic generation of c++ code from Simulink

There is the possibility to generate c++ code from the Simulink models using [Simulink coder](https://www.mathworks.com/products/simulink-coder.html) (**available only for the [floating-base-balancing-torque-control](controllers/floating-base-balancing-torque-control)**). The repositiory that contains the generated c++ code is named [autogenerated-whole-body-controllers](https://github.com/robotology-playground/autogenerated-whole-body-controllers). Documentation on how to generate the code is available in the repository [wiki](https://github.com/robotology-playground/autogenerated-whole-body-controllers/wiki/How-to-generate-code-from-a-Simulink-model).

### Static GUI for Simulink

When used for controlling real platforms, heavy Simulink models may violate the user-defined simulation time step, see also [this issue](https://github.com/robotology/wb-toolbox/issues/160). It seems a source of delay is the run-time update of the Simulink interface. For this reason, a [static GUI for running the models](library/matlab-gui) has been developed. If you want to run Simulink with the static GUI, run the [startModelWithStaticGui](controllers/floating-base-balancing-torque-control/startModelWithStaticGui.m) script.

## Where do I find legacy materials?

Official legacy repositories are: [mex-wholebodymodel](https://github.com/robotology/mex-wholebodymodel) and [WBI-Toolbox-controllers](https://github.com/robotology-legacy/WBI-Toolbox-controllers). **Note**: these legacy repos contain undocumented/outdated code, and duplicated or not tested matlab functions. They also contain original code that has been tested on the robot in the past and then never used again, or code that will be ported in the main repository in the future.

- [exploit friction and walking controller](https://github.com/robotology-legacy/WBI-Toolbox-controllers/tree/whole-body-controllers-legacy/controllers/legacy)
- [seesaw controller](https://github.com/robotology-legacy/WBI-Toolbox-controllers/tree/whole-body-controllers-legacy/controllers/legacy)
- [automatic gain tuning](https://github.com/robotology/mex-wholebodymodel/tree/master/controllers/torqueBalancingGainTuning)
- [elastic joints control](https://github.com/robotology/mex-wholebodymodel/tree/master/controllers/torqueBalancing_JE)
- [walkman control](https://github.com/robotology-legacy/WBI-Toolbox-controllers/tree/whole-body-controllers-legacy/controllers/legacy/torqueBalancing-walkman) and [walkman control-matlab](https://github.com/robotology/mex-wholebodymodel/tree/master/controllers/torqueBalancingWalkman)
- [joint-space control and centroidal transformation](https://github.com/robotology/mex-wholebodymodel/tree/master/controllers/torqueBalancingJointControl)

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
