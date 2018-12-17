%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Save and close the Simulink model through Matlab command line.
% It also closes the associate GUI

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('[closeModel]: closing the model...')

save_system('torqueBalancingStandup.mdl');
close_system('torqueBalancingStandup.mdl');
close all

% remove paths
rmpath('../../library/matlab-gui');
rmpath('./src-gui');

disp('[closeModel]: done')