%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RUN THIS SCRIPT TO REMOVE LOCAL PATHS ADDED WHEN RUNNING THE
% CONTROLLER.
%
% In the Simulink model, this script is run every time the user presses
% the 'terminate' button.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rmpath('./src/')

% Create a folder for collecting data
if Config.SAVE_WORKSPACE

    if (~exist(['experiments',date],'dir'))

        mkdir(['experiments',date]);
    end

    matFileList = dir(['./experiments',date,'/*.mat']);
    c            = clock;

    save(['./experiments',date,'/exp_',num2str(c(4)),'-',num2str(c(5)),'.mat'])
end

% Compare the Yarp time with the Simulink time to catch if the required
% integration time step is respected during the simulation
if Config.CHECK_INTEGRATION_TIME && exist('yarp_time','var')

    sim_time = yarp_time.time;

    % normalize the yarp time over the first value (at t_sim = 0)
    yarp_time0 = yarp_time.signals.values - yarp_time.signals.values(1);

    % fast check of yarp time vs. Simulink time
    if Config.PLOT_INTEGRATION_TIME

        figure
        hold on
        plot(yarp_time0,0:length(yarp_time0)-1,'o')
        plot(sim_time,0:length(sim_time)-1,'o-')
        xlabel('Time [s]')
        ylabel('Iterations')
        grid on
        legend('Yarp Time','Simulink Time')
        title('Yarp time vs. Simulink time')
    end

    % number of times the real time step was bigger than twice the
    % desired time step value
    numOfTimeStepViolations = sum(diff(yarp_time0) > 2*Config.tStep);

    if numOfTimeStepViolations > 1 && numOfTimeStepViolations <= 50

        warning(['The time step tolerance of ', num2str(Config.tStep), '[s] has been violated at least once.'])

    elseif numOfTimeStepViolations > 50

        warning(['The time step tolerance of ', num2str(Config.tStep), '[s] has been violated  more than 50 times.'])
    end
end

% If a joint hits the limits or an encoder spike is detected, print a
% warning message displaying the name of the joint
if Config.EMERGENCY_STOP_WITH_ENCODER_SPIKES && exist('res_check_spikes','var')

    if ~isempty(res_check_spikes.signals.values)

        checkEachJoint = res_check_spikes.signals.values(end,:);

        for k = 1:length(checkEachJoint)

            if checkEachJoint(k)

                warning(['A spike occurred on joint ', WBTConfigRobot.ControlledJoints{k}])
            end
        end
    end
end
if Config.EMERGENCY_STOP_WITH_JOINTS_LIMITS && exist('res_check_range','var')

    if ~isempty(res_check_range.signals.values)

        checkEachJoint = res_check_range.signals.values(end,:);

        for k = 1:length(checkEachJoint)

            if checkEachJoint(k)

                warning(['Joint ', WBTConfigRobot.ControlledJoints{k}, ' exited the range'])
            end
        end
    end
end
