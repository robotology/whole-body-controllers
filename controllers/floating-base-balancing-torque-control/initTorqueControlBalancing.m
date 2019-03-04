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

% NOTE: THIS SCRIPT IS RUN AUTOMATICALLY WHEN THE USER STARTS THE ASSOCIATED
% SIMULINK MODEL. NO NEED TO RUN THIS SCRIPT EVERY TIME.
clearvars -except sl_synch_handles simulinkStaticGUI
clc

% Add path to local source code
addpath('./src/')

%% GENERAL SIMULATION INFO
%
% If you are simulating the robot with Gazebo, remember that it is required
% to launch Gazebo as follows:
% 
%     gazebo -slibgazebo_yarp_clock.so
% 
% and properly set the environmental variable YARP_ROBOT_NAME in the .bashrc.

% Simulation time in seconds. For long simulations, put an high number 
% (not inf) for allowing automatic code generation
Config.SIMULATION_TIME = 600000;

% Controller period [s]
Config.tStep           = 0.01;

%% PRELIMINARY CONFIGURATION
%
% DEMO_TYPE: defines the kind of demo that will be performed.
%
% 'YOGA': the robot will perform the YOGA++ demo (highly dynamic movements
%         while balancing on one foot and two feet)
%   
% 'COORDINATOR': the robot can either balance on two feet or move from left
%                to right follwing a desired center-of-mass trajectory.
% 
DEMO_TYPE                     = 'YOGA';

% Config.SCOPES: debugging scopes activation
Config.SCOPES_WRENCHES        = true;
Config.SCOPES_GAIN_SCHE_INFO  = true;
Config.SCOPES_MAIN            = true;
Config.SCOPES_QP              = true;

% DATA PROCESSING
%
% Save the Matlab workspace after stopping the simulation
Config.SAVE_WORKSPACE         = false;

% Verify that the integration time has been respected during the simulation
Config.CHECK_INTEGRATION_TIME = true;
Config.PLOT_INTEGRATION_TIME  = false;

% Run robot-specific configuration parameters
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configRobot.m')); 
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/configStateMachine.m')); 
run(strcat('app/robots/',getenv('YARP_ROBOT_NAME'),'/gainsAndReferences.m')); 

% Deactivate/activate the internal coordinator
if strcmpi(DEMO_TYPE, 'COORDINATOR')

    Config.COORDINATOR_DEMO = true;
    
elseif strcmpi(DEMO_TYPE, 'YOGA')
    
    Config.COORDINATOR_DEMO = false;
end