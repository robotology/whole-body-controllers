% INITSTATEMACHINEWALKING initializes the robot configuration for running
%                         'MPC_WALKING' demo. 
%
% USAGE: please note that this function is automatically executed when
%        running the Simulink model.
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---

% OVERWRITE SOME CONFIGURATION PARAMETERS FOR RUNNING THE MPC-WALKING DEMO
if strcmp(DEMO_TYPE,'MPC_WALKING')

    % Joint torque saturation
    Sat.tau_max = 60; % [Nm]

    % Saturation on torque derivative (for QP solver)
    Sat.tauDot_max = 10000;

    % Saturation on state jerk (for QP based inverse kinematics)
    Sat.nuDDot_max = 10000;

    % Weight for the joint minimization task
    Sat.weight_tau = 0.01;

    % Damping for the pseudoinverse used for computing the floating base velocity
    Sat.pinvDamp_nu_b = 1e-6;

    %% Smoothing of reference trajectories

    % If true, reference trajectories are smoothed internally
    Config.SMOOTH_COM_REF      = false;
    Config.SMOOTH_LFOOT_POS    = false;
    Config.SMOOTH_RFOOT_POS    = false;
    Config.SMOOTH_LFOOT_ORIENT = false; 
    Config.SMOOTH_RFOOT_ORIENT = false; 
    Config.SMOOTH_ROT_TASK_REF = false;
    Config.SMOOTH_JOINT_REF    = false; 

    % Smoothing time for tasks and joints references [s]
    Config.smoothingTime_CoM    = [2;2;2];
    Config.smoothingTime_LFoot  = [2;2;2];
    Config.smoothingTime_RFoot  = [2;2;2];
    Config.smoothingTime_joints = [2;2;2];

    % Gains that will influence the smoothing of reference orientations. The
    % higher, the faster. Only positive or null values.
    Config.LFoot_Kp_smoothing    = [1;1;1];
    Config.LFoot_Kd_smoothing    = [1;1;1];
    Config.RFoot_Kp_smoothing    = [1;1;1];
    Config.RFoot_Kd_smoothing    = [1;1;1];
    Config.rot_task_Kp_smoothing = [1;1;1];
    Config.rot_task_Kd_smoothing = [1;1;1];

    % Smoothing time for gain scheduling [s].
    Config.smoothingTimeGains    = [1;1;1];
   
    %% Gains matrices

    % CoM position and velocity gains
    Gains.Kp_CoM = [50, 50, 50; ...  % state = 1 two feet balancing
                    50, 50, 50; ...  % state = 2 left foot balancing
                    50, 50, 50];     % state = 3 right foot balancing
                
    Gains.Kd_CoM = 2*sqrt(Gains.Kp_CoM);

    % Feet position and velocity gains
    Gains.Kp_LFoot = [50, 50, 50, 20, 20, 20; ... % state = 1 two feet balancing
                      50, 50, 50, 20, 20, 20; ... % state = 2 left foot balancing
                      50, 50, 50, 20, 20, 20];    % state = 3 right foot balancing
              
    Gains.Kd_LFoot = 2*sqrt(Gains.Kp_LFoot);

    Gains.Kp_RFoot = [50, 50, 50, 20, 20, 20; ... % state = 1 two feet balancing
                      50, 50, 50, 20, 20, 20; ... % state = 2 left foot balancing
                      50, 50, 50, 20, 20, 20];    % state = 3 right foot balancing

    Gains.Kd_RFoot = 2*sqrt(Gains.Kp_RFoot); 

    % Root link orientation and angular velocity gains
    Gains.Kp_rot_task = [20, 20, 20; ...  % state = 1 two feet balancing
                         20, 20, 20; ...  % state = 2 left foot balancing
                         20, 20, 20];     % state = 3 right foot balancing
                 
    Gains.Kd_rot_task =  2*sqrt(Gains.Kp_rot_task); 

    % Joint position and velocity gains
    
                        % torso      % left arm      % right arm     % left leg               % right leg                                   
    Gains.impedances = [20  20  20,  10  10  10  8,  10  10  10  8,  30  30  30  60  10  10,  30  30  30  60  10  10;  ... % state = 1 two feet balancing          
                        20  20  20,  10  10  10  8,  10  10  10  8,  30  30  30  60  10  10,  30  30  30  60  10  10;  ... % state = 2 left foot balancing
                        20  20  20,  10  10  10  8,  10  10  10  8,  30  30  30  60  10  10,  30  30  30  60  10  10]; ... % state = 3 right foot balancing
                     
    Gains.dampings   = zeros(size(Gains.impedances));
    
end

    