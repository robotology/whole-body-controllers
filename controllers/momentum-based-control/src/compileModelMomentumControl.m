%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Compile the Simulink model through Matlab command line.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp('Compiling the model...')

pause(1.5)

try
    torqueBalancingYoga([], [], [], 'compile')
    torqueBalancingYoga([], [], [], 'term')

catch ME
    
    errorMessages = ME;
end

clc

disp('Compilation done.')
warning('The model will anyways start with FEW SECONDS OF DELAY after pressing the ''Start Model'' button.')