%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TEMPLATE FOR THE CLOSEMODEL FCN (TO BE USED IN THE STATIC GUI)

% Save and close the Simulink model through Matlab command line.
% It also closes the associate static GUI

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('[closeModel]: closing the Simulink model...')

% save and close the Simulink model
save_system(MODEL_NAME_STRING);
close_system(MODEL_NAME_STRING);

% close all figures
close all

% remove paths (optional)
rmpath(PATH_TO_BE_REMOVED);

disp('[closeModel]: done.')