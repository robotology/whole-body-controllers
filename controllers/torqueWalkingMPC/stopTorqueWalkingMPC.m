%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RUN THIS SCRIPT TO REMOVE LOCAL PATHS ADDED WHEN RUNNING THE
% CONTROLLER.
%
% In the Simulink model, this script is run every time the user presses
% the terminate button.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rmpath(genpath('./src/'))
rmpath(genpath('../../library'));