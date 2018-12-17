%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% /**
%  * Copyright (C) 2016 CoDyCo
%  * @author: Daniele Pucci, Gabriele Nava
%  * Permission is granted to copy, distribute, and/or modify this program
%  * under the terms of the GNU General Public License, version 2 or any
%  * later version published by the Free Software Foundation.
%  *
%  * A copy of the license can be found at
%  * http://www.robotcub.org/icub/license/gpl.txt
%  *
%  * This program is distributed in the hope that it will be useful, but
%  * WITHOUT ANY WARRANTY; without even the implied warranty of
%  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
%  * Public License for more details
%  */
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% In the Simulink model, this script is run every time the user presses
% the 'start' button.
clearvars -except sl_synch_handles torqueBalGUI
clc

% Import +wbc scope and add path to "src" folder
import wbc.*
addpath('./src/')

%% GENERAL SIMULATION INFO
% If you are simulating the robot with Gazebo, 
% remember that you have to launch Gazebo as follow:
% 
% gazebo -slibgazebo_yarp_clock.so
% 
% and set the environmental variable YARP_ROBOT_NAME in the .bashrc file.

% Simulation time in seconds
Config.SIMULATION_TIME = 600000; % high number (not inf) for automatic code generation

%% PRELIMINARY CONFIGURATIONS 
% Sm.SM_TYPE: defines the kind of state machines that can be chosen.
%
% 'YOGA': the robot will perform the YOGA++ demo. The associated
%         configuration parameters can be found under the folder:
%
%         app/robots/YARP_ROBOT_NAME/initStateMachineYoga.m
%   
% 'COORDINATOR': the robot can either stay still, or follow a desired
%                center-of-mass trajectory. The associated configuration 
%                parameters can be found under the folder:
%
%                app/robots/YARP_ROBOT_NAME/initRefGen.m
% 
SM_TYPE                       = 'YOGA';

% Config.SCOPES: if set to true, all visualizers for debugging are active
Config.SCOPES_ALL             = true;

% You can also activate only some specific debugging scopes
Config.SCOPES_EXT_WRENCHES    = false;
Config.SCOPES_GAIN_SCHE_INFO  = false;
Config.SCOPES_MAIN            = false;
Config.SCOPES_QP              = false;
Config.SCOPES_INERTIA         = true;

% DATA PROCESSING
%
% If Config.SAVE_WORKSPACE = True, every time the simulink model is run the
% workspace is saved after stopping the simulation
Config.SAVE_WORKSPACE         = false;

% If CHECK_INTEGRATION_TIME = True, after stopping the simulation the
% Simulink time is compared with the Yarp time to check if the desired
% integration time step is respected
Config.CHECK_INTEGRATION_TIME = true;
Config.PLOT_INTEGRATION_TIME  = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CONFIGURATIONS COMPLETED: loading gains and parameters for the specific robot
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Controller period [s]
Config.Ts = 0.01; 

% Run robot-specific configuration parameters
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configRobot.m')); 

% Run demo-specific configuration parameters
Sm.SM_MASK_COORDINATOR = bin2dec('001');
Sm.SM_MASK_YOGA        = bin2dec('010');

if strcmpi(SM_TYPE, 'COORDINATOR')
    
    Sm.SM_TYPE_BIN = Sm.SM_MASK_COORDINATOR;
    demoSpecificParameters = fullfile('app/robots',getenv('YARP_ROBOT_NAME'),'initCoordinator.m');
    run(demoSpecificParameters);
    
elseif strcmpi(SM_TYPE, 'YOGA')
    
    Sm.SM_TYPE_BIN = Sm.SM_MASK_YOGA;
    demoSpecificParameters = fullfile('app/robots',getenv('YARP_ROBOT_NAME'),'initStateMachineYoga.m');
    run(demoSpecificParameters);
end

% Compute contact constraints (friction cone, unilateral constraints)
[ConstraintsMatrix,bVectorConstraints] = constraints(forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,feet_size,fZmin);