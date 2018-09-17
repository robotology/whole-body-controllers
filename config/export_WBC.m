%% export_WBC.m
%
%  Export Simulink models to a previous supported version. Currently, 
%  the default version is MATLAB 2017b. IF YOU OWE A MATLAB VERSION GREATER 
%  THAN 2017B, REMEMBER TO EXPORT ALL MODELS TO THE DEFAULT SUPPORTED 
%  VERSION BEFORE PUBLISHING ANY MODIFICATIONS, OTHERWISE OTHER USERS MAY 
%  NOT BE ABLE TO USE THE TOOLBOX!
%
clc
clear 
close all

fprintf('\nWholeBodyControl Toolbox\n');
fprintf('\nExport project to a previous Matlab version\n');
fprintf('\nOldest supported version: R2014a\n');

fprintf('\n######################################################\n');
fprintf('\nWarning: this function exports only Simulink models.\n');
fprintf('\nIf a .m file requires a dependency (toolbox, new matlab version)\n');
fprintf('\nthat is not compatible with your installation, the WBC toolbox\n');
fprintf('\nmay not work properly. Consider running also tests after the export.\n');
fprintf('\n######################################################\n\n');

% list of all simulink models (in this repo only .mdl are allowed) in the project
mdlList    = dir('../**/*.mdl');

% ignore models in the "legacy" folder (if there is any)
cont       = 1;

if exist('../legacy','dir')
    
    for j = 1:size(mdlList,1)
    
        parsedString = strsplit(mdlList(j).folder,'/');
        
        if sum(strcmp(parsedString, 'legacy')) <= 0
            
            mdlList_red(cont) = mdlList(j);  %#ok<SAGROW>
            cont              = cont +1;
        end
    end
else    
    mdlList_red = mdlList;
end

% matlab version to which export all models
matlabVer   = input('Specify the Matlab version to export models (format: R20XXx) ','s');

%% Verify matlab version

% latest version: R2017b
matlabVer_list     = {'R2014a','R2014b','R2015a','R2015b','R2016a','R2016b','R2017a','R2017b'};

% associated Simulink version
simulinkVer_number = {'8.3','8.4','8.5','8.6','8.7','8.8','8.9','9.0'};

% installed Simulink version
currentSimulinkVer = ver('Simulink');

%% Debug messages
matlabVer_found = 0;

for k = 1:length(matlabVer_list)
    
    if strcmp(matlabVer_list{k},matlabVer)
        
        fprintf(['\nExporting files into Matlab ',matlabVer,'\n\n']);
        matlabVer_found = k;
    end
end

if matlabVer_found == 0
    
    error('Version name does not correspond to any known Matlab release.');
end

if isempty(currentSimulinkVer)
    
    error('Simulink not found Verify your Simulink version.');

elseif str2double(simulinkVer_number{matlabVer_found}) > str2double(currentSimulinkVer.Version)
    
    error('It is not possible to export models to Simulink versions higher than the one installed on your system.');
end

% ask the user if it is necessary to also create a legacy copy of the 
% Simulink models with the oldest Matlab version
exportLegacyVersion = input(['Would you also like to export a legacy copy of all models with Matlab ' matlabVer_list{1}, '? Y/N '],'s');
exportInLegacy      = false;

if strcmp(exportLegacyVersion,'y') || strcmp(exportLegacyVersion,'Y')
   
    exportInLegacy = true;
    
    if  ~exist('../legacy','dir')        
        mkdir('../legacy');
    end
end

%% Load the Simulink models and export to previous versions
mdlVer_list     = {'R2014A_MDL','R2014B_MDL','R2015A_MDL','R2015B_MDL','R2016A_MDL','R2016B_MDL','R2017A_MDL','R2017B_MDL'};

% close all open models (if there is any)
close_system('',0)

for k = 1:length(mdlList_red)
    
    fprintf(['\nLOADED MODEL: ' mdlList_red(k).name '\n']);
    open_system([mdlList_red(k).folder,'/',mdlList_red(k).name],'loadonly');
        
    % save the model in a temporary copy. Then, export the copy into
    % the previous version, by overwriting the original model
    fprintf('\n saving a temporary copy of the model \n\n');
    save_system([mdlList_red(k).folder,'/',mdlList_red(k).name], [mdlList_red(k).folder,'/',mdlList_red(k).name(1:end-4),'_temp.mdl']);
    close_system([mdlList_red(k).folder,'/',mdlList_red(k).name],0);
        
    open_system([mdlList_red(k).folder,'/',mdlList_red(k).name(1:end-4),'_temp.mdl'],'loadonly');
        
    % do not export if simulink models are already at the required version
    if str2double(simulinkVer_number{matlabVer_found}) == str2double(currentSimulinkVer.Version)
       
       fprintf('\n model is already at the required version. \n');  
    else     
       save_system([mdlList_red(k).folder,'/',mdlList_red(k).name(1:end-4),'_temp.mdl'], [mdlList_red(k).folder,'/',mdlList_red(k).name], 'ExportToVersion', mdlVer_list{matlabVer_found});
    end
        
    % save a copy in legacy folder
    if exportInLegacy 
            
        if str2double(simulinkVer_number{1}) == str2double(currentSimulinkVer.Version)
            save_system([mdlList_red(k).folder,'/',mdlList_red(k).name(1:end-4),'_temp.mdl'], ['../legacy/',mdlList_red(k).name(1:end-4),'_legacy.mdl']);
        else
            save_system([mdlList_red(k).folder,'/',mdlList_red(k).name(1:end-4),'_temp.mdl'], ['../legacy/',mdlList_red(k).name(1:end-4),'_legacy.mdl'], 'ExportToVersion', mdlVer_list{1});
        end
    end
       
    % closing the temporary model
    fprintf('\n closing the model \n');
    close_system([mdlList_red(k).folder,'/',mdlList_red(k).name(1:end-4),'_temp.mdl']);
        
    % delete the temporary model
    delete([mdlList_red(k).folder,'/',mdlList_red(k).name(1:end-4),'_temp.mdl'])
end
    
fprintf('\nDone.\n');
