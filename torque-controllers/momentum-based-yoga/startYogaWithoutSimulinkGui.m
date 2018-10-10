%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% RUN THIS SCRIPT TO USE SIMULINK WITHOUT OPENING THE SIMULINK GUI

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables
clc

% add the path to the static gui and to some utility functions
addpath('../../library/matlab-gui');
addpath('./src-gui');

disp('[startModel]: loading the model...')

% open the model
open_system('torqueBalancingYoga.mdl','loadonly');

% add message to tell the user that the model has been opened correctly
disp('[startModel]: model loaded correctly')

% add warning to warn the user NOT to close the GUI 
warning('DO NOT CLOSE the GUI. The model won''t be closed! Use "Close Model" instead.')

% check if the GUI is correctly opened
if ~exist('sl_synch_handles', 'var')
    
    error('The GUI did not load correctly. Run "closeModel.m" or restart Matlab')
end