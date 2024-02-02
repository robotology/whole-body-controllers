%% exportWBCto2019b.m
%
%  Export Simulink models to a previous supported version. Currently,
%  the default version is MATLAB 2019b. IF YOU OWE A MATLAB VERSION GREATER
%  THAN 2019B, REMEMBER TO EXPORT ALL MODELS TO THE DEFAULT SUPPORTED
%  VERSION BEFORE PUBLISHING ANY MODIFICATIONS, OTHERWISE OTHER USERS MAY
%  NOT BE ABLE TO USE THE MODELS!
%
clc
clear
close all

fprintf('\nwhole-body-controllers\n');
fprintf('\nExport project to a previous Matlab version\n');
fprintf('\nWBC Default version: R2019b\n');

fprintf('\n######################################################\n');
fprintf('\nWarning: this function exports only Simulink models.\n');
fprintf('\nIf a .m file requires a dependency that is not\n');
fprintf('\ncompatible with your installation, the WBC controllers\n');
fprintf('\nmay still not work properly.\n');
fprintf('\n######################################################\n\n');

% list of all simulink mdl and slx in the project
mdlList   = dir('../**/*.mdl');
slxList   = dir('../**/*.slx');

% matlab and simulink version to which export all models
matlabVer = 'R2019b';
simulinkVer = '10.0';
mdlVer = 'R2019B_MDL';
slxVer = 'R2019B_SLX';

% installed matlab and simulink versions
currentMatlabVer_struct = ver('matlab');
currentMatlabVer = currentMatlabVer_struct.Release(2:end-1);
currentSimulinkVer = ver('Simulink');

%% Check versions
if isempty(currentSimulinkVer)

    error('Simulink not found. Verify your Simulink version.');

elseif str2double(simulinkVer) > str2double(currentSimulinkVer.Version)

    error('It is not possible to export models to Simulink versions higher than the one installed on your system.');
end

%% Load the Simulink models and export to previous version

% close all open models (if there is any)
close_system('',0)

for k = 1:length(mdlList)

    fprintf(['\nLOADED MDL FILE: ' mdlList(k).name '\n']);
    open_system([mdlList(k).folder,'/',mdlList(k).name],'loadonly');

    % save the model in a temporary copy. Then, export the copy into
    % the previous version, by overwriting the original model
    fprintf('\n saving a temporary copy of the model \n\n');
    save_system([mdlList(k).folder,'/',mdlList(k).name], [mdlList(k).folder,'/',mdlList(k).name(1:end-4),'_temp.mdl']);
    close_system([mdlList(k).folder,'/',mdlList(k).name],0);
    open_system([mdlList(k).folder,'/',mdlList(k).name(1:end-4),'_temp.mdl'],'loadonly');

    % do not export if simulink models are already at the required version
    if str2double(simulinkVer) == str2double(currentSimulinkVer.Version)

        fprintf('\n model is already at the required version. \n');
    else
        save_system([mdlList(k).folder,'/',mdlList(k).name(1:end-4),'_temp.mdl'], [mdlList(k).folder,'/',mdlList(k).name], 'ExportToVersion', mdlVer);
    end

    % closing the temporary model
    fprintf('\n closing the model \n');
    close_system([mdlList(k).folder,'/',mdlList(k).name(1:end-4),'_temp.mdl']);

    % delete the temporary model
    delete([mdlList(k).folder,'/',mdlList(k).name(1:end-4),'_temp.mdl'])
end
for k = 1:length(slxList)

    fprintf(['\nLOADED SLX FILE: ' slxList(k).name '\n']);
    open_system([slxList(k).folder,'/',slxList(k).name],'loadonly');

    % save the model in a temporary copy. Then, export the copy into
    % the previous version, by overwriting the original model
    fprintf('\n saving a temporary copy of the model \n\n');
    save_system([slxList(k).folder,'/',slxList(k).name], [slxList(k).folder,'/',slxList(k).name(1:end-4),'_temp.slx']);
    close_system([slxList(k).folder,'/',slxList(k).name],0);
    open_system([slxList(k).folder,'/',slxList(k).name(1:end-4),'_temp.slx'],'loadonly');

    % do not export if simulink models are already at the required version
    if str2double(simulinkVer) == str2double(currentSimulinkVer.Version)

        fprintf('\n model is already at the required version. \n');
    else
        save_system([slxList(k).folder,'/',slxList(k).name(1:end-4),'_temp.slx'], [slxList(k).folder,'/',slxList(k).name], 'ExportToVersion', slxVer);
    end

    % closing the temporary model
    fprintf('\n closing the model \n');
    close_system([slxList(k).folder,'/',slxList(k).name(1:end-4),'_temp.slx']);

    % delete the temporary model
    delete([slxList(k).folder,'/',slxList(k).name(1:end-4),'_temp.slx'])
end

fprintf('\nDone.\n');
