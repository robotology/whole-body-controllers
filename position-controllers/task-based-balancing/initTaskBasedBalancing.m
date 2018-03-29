% INITTASKBASEDBALANCING initializes the robot user defined configuration, gains and
%                        regularization parameters for the Simulink balancing controller.
%
% USAGE: please note that this function is automatically executed when
%        running or compiling the Simulink model.
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
clc
clear variables
close all

% NOTE: if you are simulating the robot with Gazebo, remember that you have
% to launch Gazebo as follow:
% 
%     gazebo -slibgazebo_yarp_clock.so
%
% Set the YARP_ROBOT_NAME environmental variable

setenv('YARP_ROBOT_NAME','icubGazeboSim');

% Simulation time
Config.t_end                     = inf; % [s]

% VISUALIZATION SETUP
%
% Activate all scopes in the model for visualization and debug
Config.SCOPES_ALL                = true;


% Activate scopes for visualizing inverse kinematics results
Config.SCOPES_INVERSE_KINEMATICS = false;

% Activate scopes for visualizing the robot state
Config.SCOPES_ROBOT_STATE        = false;

% Activate scopes for visualizing the smoothed reference orientations
Config.SCOPES_SMOOTH_ORIENT      = false;

% Activate scopes for visualizing the control gains
Config.SCOPES_GAIN_SCHEDULING    = false;

% Config.CHECK_LIMITS: if set to true, the controller will stop as soon as 
% any of the joint limit is touched. 
Config.CHECK_LIMITS              = false;

% DATA PROCESSING
%
% Save workspace variables when the simulation is finished
Config.SAVE_WORKSPACE            = true;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Advanced setup - do not change these parameters unless you know what you're doing
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Add path to the src folder and subfolders
addpath(genpath('./src'));
addpath(genpath('../../library'));

% Simulation step (fixed step integrator)
Config.t_step                    = 0.01; % [s]

%% STARTUP PROCEDURE
%
% A file called configRobot.m which contains a list of robot specific
% parameters is run. Then, the configuration file corresponding to the demo is run.
%
configRobotFCN                   = fullfile('app/robots', getenv('YARP_ROBOT_NAME'),'configRobot.m');
run(configRobotFCN);

% Run configuration script for walking with MPC
stateMachineWalkingFCN           = fullfile('app/robots', getenv('YARP_ROBOT_NAME'),'initStateMachineBalancing.m');
run(stateMachineWalkingFCN);

disp('Initialize taskBasedBalancing')
disp(['Selected robot: ',getenv('YARP_ROBOT_NAME')])
