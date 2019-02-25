%% startup_WBC.m
%
%  Run this script once to permanently add the matlab-wbc library to your MATLAB path. 

fprintf('\n## whole-body-controllers ##\n');
fprintf('\nAdding "matlab-wbc" library to pathdef.m...\n\n');

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
