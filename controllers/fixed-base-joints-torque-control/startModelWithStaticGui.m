%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% RUN THIS SCRIPT TO USE SIMULINK WITH THE STATIC SIMULINK GUI

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear variables
clc

% add the path to the static gui and to some utility functions
addpath('../../library/matlab-gui');
addpath('./src-static-gui');

disp('[startModel]: loading the model...')

% open the model
open_system('jointsControl.mdl','loadonly');

% add message to tell the user that the model has been opened correctly
disp('[startModel]: model loaded correctly')
disp('[startModel]: the "Start Model" button is enabled only after compiling the model.')

% add warning to warn the user NOT to close the GUI 
warning('DO NOT CLOSE the GUI. The model won''t be closed! Use "Exit Model" button instead.')

% check if the GUI is correctly opened
if ~exist('sl_synch_handles', 'var')
    
    error('The GUI did not load correctly, or it is already opened. Close the GUI, run "closeModel.m" or restart Matlab')
end