%% startup_WBC.m
%
%  Run this script once to permanently add the matlab-wbc library to your MATLAB path. 

clc
fprintf('\n## whole-body-controllers ##\n');
fprintf('\nAdding "matlab-wbc" library to your pathdef.m...\n\n');

% path to whole-body-controllers
pathToWBC = pwd;
pathToWBC = pathToWBC(1:end-6);

% path to the matlab-wbc library
pathToLibrary = [pathToWBC, filesep, 'library/matlab-wbc'];

if exist(pathToLibrary, 'dir')
    
    addpath(pathToLibrary);
else
    error('Path to the "matlab-wbc" library not found or not correct.')
end

% Path to the Matlab userpath
pathToUserpath        = userpath;
pathSeparatorLocation = strfind(pathToUserpath, pathsep);

if isempty(pathToUserpath)
    
    answer = input('Empty userpath. Do you want to reset the userpath? Y/N ','s');

    if strcmpi(answer,'Y')
        
       userpath('reset');
       disp('Resetting userpath..');
       pathToUserpath        = userpath;
       pathSeparatorLocation = strfind(pathToUserpath, pathsep);
    else
       error('Please set the userpath before running this script');
    end
      
elseif size(pathSeparatorLocation, 2) > 1
    
    answer = input('Multiple userpath. Do you want to reset the userpath? Y/N ','s');

    if strcmpi(answer,'Y')
        
       userpath('reset');
       disp('Resetting userpath..');
       pathToUserpath        = userpath;
       pathSeparatorLocation = strfind(pathToUserpath, pathsep);
    else
       error('Please set a single userpath before running this script');
    end
end

% check again the userpath
if isempty(pathToUserpath)
    
    error('userpath is still empty. Please set the userpath before running this script');
    
elseif size(pathSeparatorLocation, 2) > 1
    
    error('There are still multiple userpath. Please set a single userpath before running this script');
end

if (~isempty(pathSeparatorLocation))
    
    pathToUserpath(pathSeparatorLocation) = [];
end

fprintf('Saving paths to %s\n\n', [pathToUserpath, filesep, 'pathdef.m']);

if (~savepath([pathToUserpath, filesep, 'pathdef.m']))
    
    fprintf(['A file called pathdef.m has been created in your %s folder.\n', ...
             'This should be enough to permanently add matlab-wbc to ', ...
             'your MATLAB installation.\n'], pathToUserpath);
else
    disp('There was an error generating the pathdef.m. Please manually add the matlab-wbc folder to your matlabpath');
end

% save a script named "goToWholeBodyControllers" inside the pathdef folder,
% to facilitate the user to reach the WBC working folder

% get the WBC path
[~, currentFolderName, ~] = fileparts(pwd);
currentPath               = pwd;
wbcPath                   = currentPath(1:end-length(currentFolderName));

% note: exist returns 2 if a .m file with the specified name is found
if exist([pathToUserpath, filesep, 'goToWholeBodyControllers.m'],'file') == 2
    
    fprintf('\n')
    warning(['A goToWholeBodyControllers.m file exists already in your ',pathToUserpath, ' folder,' ...
             ' and therefore it won''t be created.'])
else
    % create the script and write inside the cd command
    fid = fopen([pathToUserpath, filesep, 'goToWholeBodyControllers.m'],'w');
    fprintf(fid, ['cd ', wbcPath, ';']);
    fclose(fid);

    fprintf('\n')
    fprintf(['A file called goToWholeBodyControllers.m has also been created in your %s folder.\n', ...
             'This will help to quickly reach the WBC-project folder after ', ...
             'Matlab is launched.\n'], pathToUserpath);
end

