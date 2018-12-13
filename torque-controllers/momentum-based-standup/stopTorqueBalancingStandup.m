%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% RUN THIS SCRIPT TO REMOVE LOCAL PATHS ADDED WHEN RUNNING THE
% CONTROLLER.
%
% In the Simulink model, this script is run every time the user presses
% the terminate button.

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
rmpath('./src/')
rmpath('../../library/matlab');

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
    
    % number of times the Simulink time step was bigger than twice the
    % desired time step value
    numOfTimeStepViolations = sum(diff(sim_time) > 2*Config.Ts);
    
    if numOfTimeStepViolations > 1
        
        warning(['The time step tolerance of ', num2str(Config.Ts), '[s] has been violated at least once.'])
        
    elseif numOfTimeStepViolations > 1000
        
        warning(['The time step tolerance of ', num2str(Config.Ts), '[s] has been violated  more than 1000 times.'])
    end
end