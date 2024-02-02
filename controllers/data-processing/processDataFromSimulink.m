% This script assumes that the variables under interest of analysis are:
%
% 1) SAVED FROM SIMULINK as STRUCTURE WITH TIME format;
%
% 2) contains the suffix "_DATA" in the name.
%
% Then, the script reads the workspace (or loads a .mat file), looks for
% these variables and allows the user to choose which variable to plot.
%
%

%% DATA TO LOAD (comment if data are in the workspace)
% load('EXAMPLE.mat')

%% SETUP VISUALIZER
close all
clc

addpath('./src_plot')

%%utility variables

% labels and font size
xAxisLabel  = 'Time [s]';
lineSize    = 2;

% print figures to .png
printFigure = false;

% dock all figures together
dockFigures = true;

% desired init and end time in the figures [s]
timeInit    = 0;
timeLimit   = 120;

if dockFigures

    set(0,'DefaultFigureWindowStyle','docked')
else
    set(0,'DefaultFigureWindowStyle','normal')
end

%% Get dataset from workspace
dataVariables = whos('*DATA');
plotList      = {};

for k = 1:length(dataVariables)

    plotList{k} = dataVariables(k).name; %#ok<SAGROW>
end

[plotNumber, ~] = listdlg('PromptString',  'Choose the variables to plot:', ...
    'ListString',     plotList, ...
    'SelectionMode', 'multiple', ...
    'ListSize',       [250 150]);

% regenerate the plot list with the data selected by user
plotList = plotList(plotNumber);

%% Select which time to use for the plot (if Yarp time is available)
timeList = {'simulink time'};

if exist('time_Yarp','var')

    timeList{2} = 'yarp time';
end

if length(timeList) > 1

    [timeNumber, ~] = listdlg('PromptString',  'Choose the time vector to use:', ...
        'ListString',     timeList, ...
        'SelectionMode', 'single', ...
        'ListSize',       [250 150]);
else
    timeNumber = 1;
end

%% Process data to plot
figNumber = 0;

for k = 1:length(plotList)

    currentData = eval(plotList{k});

    if timeNumber == 1

        time = currentData.time;

    elseif timeNumber == 2

        time = time_Yarp.signals.values;
    end

    % get the user defined time length
    timeLimitIndex = sum(time <= timeLimit);
    timeInitIndex  = sum(time <= timeInit);
    tVector        = time(timeInitIndex:timeLimitIndex);

    dataStructure  = currentData.signals;
    figNumber      = figNumber + 1;

    for kk = 1:length(currentData.signals)

        valuesToPlot = squeeze(currentData.signals(kk).values);

        % twist data dimensions if necessary
        if size(valuesToPlot,1) < length(time)

            valuesToPlot = valuesToPlot';
        end

        valuesToPlot = valuesToPlot(timeInitIndex:timeLimitIndex,:);
        yAxisLabel   = currentData.signals(kk).label;

        if kk == 1

            titleLabel = plotList{k}(1:end-5);
        else
            titleLabel = '';
        end

        figure(figNumber)
        subplot(length(currentData.signals),1,kk)
        custom_plot(tVector, valuesToPlot, xAxisLabel, yAxisLabel, titleLabel, ...
            lineSize, '', figNumber, printFigure, 0);
    end
end

rmpath('./src_plot')
