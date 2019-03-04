%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Save and close the Simulink model through Matlab command line.
% It also closes the associate static GUI

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('[closeModel]: closing the Simulink model...')

% save and close the Simulink model
save_system('jointsControl.mdl');
close_system('jointsControl.mdl');

% close all figures
close all

% remove paths (optional)
rmpath('../../library/matlab-gui');
rmpath('./src-static-gui');

disp('[closeModel]: done.')