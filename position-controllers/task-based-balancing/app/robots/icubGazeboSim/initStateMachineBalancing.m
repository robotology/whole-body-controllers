% INITSTATEMACHINEBALANCING initializes the robot configuration for task
%                           based position control balancing.
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

% ROBOT SETUP 
%

% Numerical tolerance for assuming a foot on contact and for rotational PID activation
Sat.toll_feetInContact           = 0.1;
Sat.toll_useRotPID               = 0.1;

% Damping for the pseudoinverse used for computing the floating base velocity
Sat.pinvDamp_nu_b                = 0.1;

% Saturation on state jerk (for QP based inverse kinematics)
Sat.nuDDot_max                   = 10000;

%% Smoothing of reference trajectories and base 

% If true, the left foot is assumed to be in contact at t = 0
Config.LEFT_FOOT_IN_CONTACT      = true;

% If true, reference trajectories are smoothed internally
Config.SMOOTH_COM_REF            = true;
Config.SMOOTH_LFOOT_POS          = true;
Config.SMOOTH_RFOOT_POS          = true;
Config.SMOOTH_LFOOT_ORIENT       = true; 
Config.SMOOTH_RFOOT_ORIENT       = true; 
Config.SMOOTH_ROT_TASK_REF       = true;
Config.SMOOTH_JOINT_REF          = true; 

% If true, joint references are modified in order not to be in conflict with
% the Cartesian tasks. The new joint references are calculated by means of an
% integration based inverse kinematics
Config.USE_INVERSE_KINEMATICS             = true;

% If true, the output of QP solver will be forced to be continuous
Config.QP_IKIN_USE_CONTINUITY_CONSTRAINTS = false;

% Smoothing time for tasks and joints references [s]
Config.smoothingTime_CoM         = [1.5;  1.5;  1.5];
Config.smoothingTime_LFoot       = [0.5;  0.5;  0.5];
Config.smoothingTime_RFoot       = [0.5;  0.5;  0.5];
Config.smoothingTime_joints      = [0.5;  0.5;  0.5];

% Gains that will influence the smoothing of reference orientations. The
% lower are, the more smoothed is the trajectory. Only positive or null values.
Config.LFoot_Kp_smoothing        = [1;  1;  1];
Config.RFoot_Kp_smoothing        = [1;  1;  1];
Config.rot_task_Kp_smoothing     = [1;  1;  1];

% Smoothing time for gain scheduling [s].
Config.smoothingTimeGains        = [0.2;  0.2;  0.2];

% Minimum value of the vertical force at contact location for the contact
% to be considered as active 
Config.threshold_contact_twoFeet = 20;  % [N]
Config.threshold_contact_on      = 180; % [N]
Config.threshold_contact_off     = 90;  % [N]

%% Gains matrices

% CoM position and velocity gains
Gains.Kp_CoM                     = [60, 70, 70; ...  % state = 1 two feet balancing
                                    60, 70, 70; ...  % state = 2 move CoM on left foot
                                    60, 70, 70]/5;   % state = 3 left foot balancing
                
Gains.Kd_CoM                     = 2*sqrt(Gains.Kp_CoM);

% Feet position and velocity gains              
Gains.Kp_LFoot                   = [75, 75, 100, 50, 50, 50; ...   % state = 1 two feet balancing
                                    75, 75, 100, 50, 50, 50; ...   % state = 2 move CoM on left foot
                                    75, 75, 100, 50, 50, 50]/5;    % state = 3 left foot balancing
              
Gains.Kd_LFoot                   = 2*sqrt(Gains.Kp_LFoot);

Gains.Kp_RFoot                   = [75, 75, 100, 50, 50, 50; ...   % state = 1 two feet balancing
                                    75, 75, 100, 50, 50, 50; ...   % state = 2 move CoM on left foot
                                    75, 75, 100, 50, 50, 50]/5;    % state = 3 left foot balancing

Gains.Kd_RFoot                   = 2*sqrt(Gains.Kp_RFoot); 

% Root link orientation and angular velocity gains
Gains.Kp_rot_task                = [50, 50, 50; ...    % state = 1 two feet balancing
                                    50, 50, 50; ...    % state = 2 move CoM on left foot
                                    50, 50, 50]/5;     % state = 3 left foot balancing
                 
Gains.Kd_rot_task                =  2*sqrt(Gains.Kp_rot_task); 

% Joint position and velocity gains
    
                                    % torso      % left arm    % right arm   % left leg           % right leg                                   
Gains.impedances                 = [60  60  60,  50 50 50 50,  50 50 50 50,  80 70 100 60 70 70,  80 70 100 60 70 70;  ...     % state = 1 two feet balancing          
                                    60  60  60,  50 50 50 50,  50 50 50 50,  80 70 100 60 70 70,  80 70 100 60 70 70;  ...     % state = 2 move CoM on left foot
                                    60  60  60,  50 50 50 50,  50 50 50 50,  80 70 100 60 70 70,  80 70 100 60 70 70]/5; ...   % state = 3 left foot balancing
                     
Gains.dampings                   = 2*sqrt(Gains.impedances);                    

    