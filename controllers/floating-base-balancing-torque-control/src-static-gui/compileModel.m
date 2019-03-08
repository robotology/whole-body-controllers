%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Compile the Simulink model through Matlab command line.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Compiling the Simulink model...')

pause(1.5)

try
    torqueControlBalancing([], [], [], 'compile')
    torqueControlBalancing([], [], [], 'term')

catch ME
    
    errorMessages = ME;
end

clc

disp('Compilation done.')

% warning about Simulink timing
warning('The model will anyways start with FEW SECONDS OF DELAY after pressing the ''Start Model'' button.')