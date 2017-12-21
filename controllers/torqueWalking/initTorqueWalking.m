% INITTORQUEWALKING initializes the robot user defined configuration, gains and
%                   regulation parameters for the Simulink balancing controller.
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
clear all  %#ok<CLALL>
close all

% NOTE: if you are simulating the robot with Gazebo, remember that you have
% to launch Gazebo as follow:
% 
%     gazebo -slibgazebo_yarp_clock.so
%
% Set the YARP_ROBOT_NAME environmental variable

% setenv('YARP_ROBOT_NAME','iCubGenova04');
  setenv('YARP_ROBOT_NAME','icubGazeboSim');
% setenv('YARP_ROBOT_NAME','iCubGenova02');

% SELECT THE DEMO TO BE PERFORMED:
%
%    - 'COORDINATOR' = the robot will balance assuming one link is not
%                      moving from its initial position.
%
%    - 'MPC_WALKING' = the model is connected to an MPC controller which
%                      streams references and contact status for walking.
%
%    - 'EXAMPLE_STATE_MACHINE' = a simple state machine for testing the
%                                controller while executing different task.
%
DEMO_TYPE = 'MPC_WALKING';

% Simulation time
Config.t_end = inf; % [s]

% VISUALIZATION SETUP
%
% Activate all scopes in the model for visualization and debug
Config.SCOPES_ALL = true;

% Activate scopes related to forces and torques visualization
Config.SCOPES_TORQUES_AND_FORCES = false;

% Activate scopes for visualizing inverse kinematics results
Config.SCOPES_INVERSE_KINEMATICS = false;

% Activate scopes for visualizing forward kinematics results
Config.SCOPES_FORWARD_KINEMATICS = false;

% Activate scopes for visualizing the robot state
Config.SCOPES_ROBOT_STATE = false;

% Activate scopes for visualizing the smoothed reference orientations
Config.SCOPES_SMOOTH_ORIENT = false;

% SIMULATION SETUP
%
% If true, joint references are modified in order not to be in conflict with
% the Cartesian tasks. The new joint references are calculated by means of an
% integration based inverse kinematics
Config.USE_INVERSE_KINEMATICS = false;

% If true, the output of QP solver will be forced to be continuous
Config.QP_USE_CONTINUITY_CONSTRAINTS = true;
Config.QP_IKIN_USE_CONTINUITY_CONSTRAINTS = true;

% If true, the IMU orientation is used in order to estimate the
% base-to-world transformation matrix
Config.USE_IMU4EST_BASE = false;

% If true, the orientation provided by the IMU is corrected using the neck
% positions (requires the neck port to be active)
Config.CORRECT_IMU_WITH_NECK_POS = true;

% If true, IMU pitch and yaw are not considered for estimating base-to-world transform
Config.FILTER_IMU_YAW = false;
Config.FILTER_IMU_PITCH = false;

% If true, the robot will move its CoM while balancing, following a sine trajectory
Config.DEMO_MOVEMENTS = false;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Advanced setup - do not change these parameters unless you know what you're doing
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Add path to the src folder and subfolders
addpath(genpath('./src'));
addpath(genpath('../../library'));

% True if the controller is connected to Gazebo simulator. 
if strcmp(getenv('YARP_ROBOT_NAME'),'icubGazeboSim')
    
    Config.ON_GAZEBO = true;
else
    Config.ON_GAZEBO = false;
end

%% Demo type selector

% True if the walking with MPC demo is selected 
if strcmp(DEMO_TYPE,'MPC_WALKING')
    
    % Emergency stop if ports are streaming null data
    Config.CHECK_PORTS_WALKING = true;
    Config.WALKING_WITH_MPC = true;
else
    Config.CHECK_PORTS_WALKING = false;
    Config.WALKING_WITH_MPC = false;
end

% True if the example state machine demo is selected
if strcmp(DEMO_TYPE,'EXAMPLE_STATE_MACHINE')
    
    Config.STATE_MACHINE_EXAMPLE = true;
else
    Config.STATE_MACHINE_EXAMPLE = false;
end

% Simulation step (fixed step integrator)
Config.t_step = 0.01; % [s]

% Frames name list
Frames.IMU = 'imu_frame';
Frames.BASE_LINK = 'root_link';
Frames.LEFT_FOOT = 'l_sole';
Frames.RIGHT_FOOT = 'r_sole';
Frames.COM = 'com';
Frames.ROT_TASK_LINK = 'neck_2';

% STARTUP PROCEDURE
%
% Simulink model requires all parameters to be defined in order to run the
% model. Hence, the model must load every time all the parameters, even if they
% are DEMO-SPECIFIC, i.e. they just affect one demo. For this reason, all the
% demo-configuration files must be executed every time. Each specific 
% configuration file is divided into 2 parts: first, parameters which are 
% in common with another demo will be ALL OVERWRITTEN if and only if that 
% particular demo has been selected. Then, parameters which are DEMO-SPECIFIC 
% and does not affect the other demos will be anyway loaded.
% DO NOT MODIFY THIS BEHAVIOUR.
%
% Run configuration script for internal coordinator
internalCordinatorFCN = fullfile('app/robots', getenv('YARP_ROBOT_NAME'),'initCoordinator.m');
run(internalCordinatorFCN);

% Run configuration script for walking with MPC
stateMachineWalkingFCN = fullfile('app/robots', getenv('YARP_ROBOT_NAME'),'initStateMachineWalking.m');
run(stateMachineWalkingFCN);

% Run configuration script for exmple state machine
stateMachineExampleFCN = fullfile('app/robots', getenv('YARP_ROBOT_NAME'),'initStateMachineExample.m');
run(stateMachineExampleFCN);

% Ports name list (requires WBT_robotName to be setted)
Ports.LEFT_FOOT_EXT_WRENCH  = '/wholeBodyDynamics/left_leg/cartesianEndEffectorWrench:o';
Ports.RIGHT_FOOT_EXT_WRENCH = '/wholeBodyDynamics/right_leg/cartesianEndEffectorWrench:o';
Ports.IMU = ['/' WBT_robotName '/inertial'];
Ports.NECK_POS = ['/' WBT_robotName '/head/state:o'];

% Ports for connecting the model with the MPC controller
PORTS.COM_DES = '/walking-coordinator/com:o';
PORTS.Q_DES = '/walking-coordinator/joints:o';
PORTS.LFOOT_DES = '/walking-coordinator/leftFoot:o';
PORTS.RFOOT_DES = '/walking-coordinator/rightFoot:o';
PORTS.ACTIVE_CONTACTS = '/walking-coordinator/contact:o';
PORTS.LFOOT_IS_FIXED = '/walking-coordinator/leftStanding:o';
PORTS.ACK = '/walking-coordinator/done:i';

% Compute the constraint matrix and bias vector for friction and unilateral
% constraints at contact locations
[ConstraintMatrix_feet, biasVectorConstraint_feet] = constraints(forceFrictionCoefficient, numberOfPoints, torsionalFrictionCoefficient, Config.footSize, fZmin);

disp('Initialize torqueWalking')
disp(['Robot: ',getenv('YARP_ROBOT_NAME')])
disp(['Demo type: ', DEMO_TYPE])
