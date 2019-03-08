%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% TEMPLATE FOR THE COMPILEMODEL FCN (TO BE USED WITH THE STATIC GUI)

% Compile the Simulink model through Matlab command line.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
disp('Compiling the Simulink model...')

pause(1.5)

try
    SIMULINK_MODEL_NAME([], [], [], 'compile')
    SIMULINK_MODEL_NAME([], [], [], 'term')

catch ME
    
    errorMessages = ME;
end

clc

disp('Compilation done.')

% warning about Simulink timing
warning('The model will anyways start with FEW SECONDS OF DELAY after pressing the ''Start Model'' button.')