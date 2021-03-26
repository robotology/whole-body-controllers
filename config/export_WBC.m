%% export_WBC.m
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
fprintf('\nOldest supported version: R2014a\n');

fprintf('\n######################################################\n');
fprintf('\nWarning: this function exports only Simulink models.\n');
fprintf('\nIf a .m file requires a dependency that is not\n');
fprintf('\ncompatible with your installation, the WBC controllers\n');
fprintf('\nmay still not work properly.\n');
fprintf('\n######################################################\n\n');

% list of all simulink mdl and slx in the project
mdlList   = dir('../**/*.mdl');
slxList   = dir('../**/*.slx');

% matlab version to which export all models
matlabVer = input('Specify the Matlab version to export models (format: R20XXx) ','s');

%% Verify matlab version

% latest version: R2019b
matlabVer_list     = {'R2014a','R2014b','R2015a','R2015b','R2016a','R2016b','R2017a','R2017b','R2018a','R2018b','R2019a','R2019b'};

% associated Simulink version
simulinkVer_number = {'8.3','8.4','8.5','8.6','8.7','8.8','8.9','9.0','9.1','9.2','9.3','10.0'};

% installed Simulink version
currentSimulinkVer = ver('Simulink');

%% Debug messages
matlabVer_found = 0;

for k = 1:length(matlabVer_list)
    
    if strcmp(matlabVer_list{k},matlabVer)
        
        fprintf(['\nExporting files into Matlab ',matlabVer, '...','\n\n']);
        matlabVer_found = k;
    end
end

if matlabVer_found == 0
    
    error('Version name does not correspond to any known Matlab release.');
end

if isempty(currentSimulinkVer)
    
    error('Simulink not found. Verify your Simulink version.');

elseif str2double(simulinkVer_number{matlabVer_found}) > str2double(currentSimulinkVer.Version)
    
    error('It is not possible to export models to Simulink versions higher than the one installed on your system.');
end

%% Load the Simulink models and export to previous versions
mdlVer_list = {'R2014A_MDL','R2014B_MDL','R2015A_MDL','R2015B_MDL','R2016A_MDL','R2016B_MDL','R2017A_MDL','R2017B_MDL','R2018A_MDL','R2018B_MDL','R2019A_MDL','R2019B_MDL'};
slxVer_list = {'R2014A_SLX','R2014B_SLX','R2015A_SLX','R2015B_SLX','R2016A_SLX','R2016B_SLX','R2017A_SLX','R2017B_SLX','R2018A_SLX','R2018B_SLX','R2019A_SLX','R2019B_SLX'};

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
    if str2double(simulinkVer_number{matlabVer_found}) == str2double(currentSimulinkVer.Version)
       
       fprintf('\n model is already at the required version. \n');  
    else     
       save_system([mdlList(k).folder,'/',mdlList(k).name(1:end-4),'_temp.mdl'], [mdlList(k).folder,'/',mdlList(k).name], 'ExportToVersion', mdlVer_list{matlabVer_found});
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
    if str2double(simulinkVer_number{matlabVer_found}) == str2double(currentSimulinkVer.Version)
       
       fprintf('\n model is already at the required version. \n');  
    else     
       save_system([slxList(k).folder,'/',slxList(k).name(1:end-4),'_temp.slx'], [slxList(k).folder,'/',slxList(k).name], 'ExportToVersion', slxVer_list{matlabVer_found});
    end
              
    % closing the temporary model
    fprintf('\n closing the model \n');
    close_system([slxList(k).folder,'/',slxList(k).name(1:end-4),'_temp.slx']);
        
    % delete the temporary model
    delete([slxList(k).folder,'/',slxList(k).name(1:end-4),'_temp.slx'])
end
    
fprintf('\nDone.\n');
